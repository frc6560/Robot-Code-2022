// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.invoke.ConstantBootstraps;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.utility.ShootCalibrationMap;
import frc.robot.utility.Util;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotIds;
import frc.robot.Constants.ShooterCalibrations;
import frc.robot.commands.autonomous.AutonomousController;
import frc.robot.controls.manualdrive.ManualControls;
import frc.robot.subsystems.Limelight;

public class ShooterCommand extends CommandBase {

  public static interface Controls {
    double shooterHoodTest();
    double shooterTurretTest();
    boolean getAimShooter();
    boolean overrideTurretCenter();
  }

  private Shooter shooter;
  private Controls controls;
  private Limelight limelight;

  private ColorMatch colorMatch = new ColorMatch();
  private final Color blueColor = new Color(0, 0, 255);
  private final Color redColor = new Color(255, 0, 0);

  private final boolean isRedAlliance;

  private boolean missBall = false;
  private final double ballMissAngle = 10;

  private NetworkTable ntTable;
  private NetworkTableEntry ntTestRPM;
  private NetworkTableEntry ntTestHood;

  private NetworkTableEntry ntAddCalibrateButton;
  private boolean prevCalibButton = false;
  private NetworkTableEntry ntUseCalibrationMap;

  private double targetHoodPos = 0.0;
  
  private int targetBallCount = -1;
  private double doneShootingFrames = 0;

  public ShooterCommand(Shooter shooter, Controls controls, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.controls = controls;
    this.limelight = limelight;

    addRequirements(shooter, limelight);

    colorMatch.addColorMatch(blueColor);
    colorMatch.addColorMatch(redColor);

    
    this.ntTable = NetworkTableInstance.getDefault().getTable("Shooter");

    ntTestHood = ntTable.getEntry("Hood Angle");
    ntTestHood.setDouble(0.0);

    ntTestRPM = ntTable.getEntry("Target Cal RPM");
    ntTestRPM.setDouble(0.0);

    ntAddCalibrateButton = ntTable.getEntry("Save point on map");
    ntAddCalibrateButton.setBoolean(false);

    ntUseCalibrationMap = ntTable.getEntry("Use calibration map?");
    ntUseCalibrationMap.setBoolean(true);

    isRedAlliance =  NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(false);
  }
  

  public ShooterCommand(Shooter shooter, Limelight limelight, boolean shootingFar, int ballCount){ // Autonomouse
    this(shooter, new AutonomousController(shootingFar, "Shooter", "conveyor", "Intake"), limelight);
    this.targetBallCount = ballCount;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setHoodPos(0.0);
    shooter.setShooterRpm(0.0);
    shooter.resetBallCount();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // if(!RobotContainer.conveyorSensor.get()){
    //   if(colorMatch.matchClosestColor(RobotContainer.colorSensor.getColor()).color == blueColor && isRedAlliance)
    //     missBall = true;
    //   else
    //     missBall = false;
    // }

    limelight.setForceOff(!controls.getAimShooter());

    double dist = limelight.getDistance();
    if(controls.getAimShooter()){
      targetHoodPos = getShooterHoodAngle(dist);

      shooter.setShooterRpm(getShooterRpm(dist) * 3.454545457);
      
      if(targetHoodPos >= -1){
        shooter.setHoodPos(targetHoodPos);
      }

      // shooter.setTurretPos(shooter.getTurretPos() + controls.shooterTurretTest()); // manual control of turret using climb joystick (button board);
      double turrTarget = limelight.getHorizontalAngle();

      // if(missBall){
      //   if(turrTarget > 0){
      //     turrTarget -= ballMissAngle;
      //   } else{
      //     turrTarget += ballMissAngle;
      //   }
      // }
      
      if((shooter.getTurretPosDegrees() > 85 && turrTarget > 0) || (shooter.getTurretPosDegrees() < -85 && turrTarget < 0))
        turrTarget = 0;
      
      if(controls.overrideTurretCenter()) shooter.setTurretPos(0);
      else shooter.setTurretDeltaPos(turrTarget); // limelight controlled turret pos;

      // ntTestHood.setDouble(targetHoodPos);
    }else{
      // shooter.setTurretPos(0);
      shooter.setShooterRpm(0);
    }

    if(targetBallCount != -1 && shooter.getBallShotCount() >= targetBallCount) doneShootingFrames++;

    if(!prevCalibButton && ntAddCalibrateButton.getBoolean(false)){
      ShooterCalibrations.SHOOT_CALIBRATION_MAP.add(dist, new ShootCalibrationMap.Trajectory(ntTestRPM.getDouble(0.0), ntTestHood.getDouble(0.0)));
      System.out.println("added a point");
    }
    prevCalibButton = ntAddCalibrateButton.getBoolean(false);
  }

  public double getShooterRpm(double distance) {
    if(ntUseCalibrationMap.getBoolean(false)){
      ShootCalibrationMap.Trajectory traj;
      try {
        traj = Constants.ShooterCalibrations.SHOOT_CALIBRATION_MAP.get(distance);
        
      } catch (ShootCalibrationMap.OutOfBoundsException e) {
        return 0.0;
      }

      return traj.shooterRpm;
    }
    return ntTestRPM.getDouble(0.0);
    // return distance; //TODO: add function
  }

  public double getShooterHoodAngle(double distance) {
    if(ntUseCalibrationMap.getBoolean(false)){
      ShootCalibrationMap.Trajectory traj;
      try {
        traj = Constants.ShooterCalibrations.SHOOT_CALIBRATION_MAP.get(distance);
        
      } catch (ShootCalibrationMap.OutOfBoundsException e) {
        return 0.0;
      }

      return traj.hoodPos;
    }
    return ntTestHood.getDouble(0.0); 
  }

  public boolean doneShooting(){
    return doneShootingFrames > 30;
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShooterRpm(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return doneShooting();
  }
}
