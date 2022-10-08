// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.utility.ShootCalibrationMap;
import frc.robot.Constants;
import frc.robot.commands.autonomous.AutonomousController;
import frc.robot.subsystems.Limelight;

public class ShooterCommand extends CommandBase {

  public static interface Controls extends DemoControls {
    double shooterHoodTest();
    double shooterTurretTest();
    boolean getAimShooter();
    boolean overrideTurretCenter();
    boolean getConstantAiming();

    boolean getHotRPMAddition();
    boolean getHotRPMReduction();

    boolean getManualMiss();
  }
  
  public interface DemoControls {
    boolean shootClose();
    boolean shootMid();
    boolean shootFar();
    boolean shootUp();

    boolean shooterPanRight();
    boolean shooterPanLeft();
  }

  private Shooter shooter;
  private Controls controls;
  private Limelight limelight;

  private ColorMatch colorMatch = new ColorMatch();
  private final Color blueColor = new Color(0, 0, 255);
  private final Color redColor = new Color(255, 0, 0);

  private final boolean isRedAlliance;
  private boolean isAuto = false;

  private boolean missBall = false;
  private final double ballMissRPM = 500;
 
  private NetworkTable ntTable;
  private NetworkTable ntTableClimb;
  private NetworkTableEntry ntTestRPM;
  private NetworkTableEntry ntTestHood;

  private NetworkTableEntry ntUseCalibrationMap;

  private NetworkTableEntry hotRPMAddition;
  private NetworkTableEntry hotRPMReduction;

  private NetworkTableEntry ntTeleopBuff;


  private NetworkTableEntry ntDemoMode;

  private final double IDLE_RPM = 1000;
  private final double AutoBaseRPMBuff = 100;
  private double TeleOpBaseRPMBuff = 100;

  private double targetHoodPos = 0.0;
  
  private int targetBallCount = -1;
  private double doneShootingFrames = 0;

  private Debouncer debouncer = new Debouncer(2, DebounceType.kFalling);

  private double rpmBuff;
  private final double rpmBuffZeta = 1;

  public ShooterCommand(Shooter shooter, Controls controls, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.controls = controls;
    this.limelight = limelight;

    addRequirements(shooter, limelight);

    colorMatch.addColorMatch(blueColor);
    colorMatch.addColorMatch(redColor);

    
    this.ntTable = NetworkTableInstance.getDefault().getTable("Shooter");
    this.ntTableClimb = NetworkTableInstance.getDefault().getTable("Climb");

    ntTestHood = ntTable.getEntry("Hood Angle");
    ntTestHood.setDouble(0.0);

    ntTestRPM = ntTable.getEntry("Target Cal RPM");
    ntTestRPM.setDouble(0.0);

    ntUseCalibrationMap = ntTable.getEntry("Use calibration map?");
    ntUseCalibrationMap.setBoolean(true);

    hotRPMAddition = ntTable.getEntry("hot RPM Addition");
    hotRPMAddition.setDouble(35.0);

    ntTeleopBuff = ntTable.getEntry("Teleop RPM Buff");
    ntTeleopBuff.setDouble(0);

    ntDemoMode = ntTable.getEntry("DEMO MODE");
    ntDemoMode.setBoolean(false);
    

    isRedAlliance =  NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(false);

    this.isAuto = false;
  }
  

  public ShooterCommand(Shooter shooter, Limelight limelight, boolean shootingFar, int ballCount){ // Autonomouse
    this(shooter, new AutonomousController(shootingFar, "Shooter", "conveyor", "Intake"), limelight);
    this.targetBallCount = ballCount;
    this.isAuto = true;
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
    //   ColorMatchResult closestColor = colorMatch.matchClosestColor(RobotContainer.colorSensor.getColor());
    //   // System.out.println("Ball Color Detected: (" + closestColor.color.red + ", " +closestColor.color.green + ", " + closestColor.color.blue +")");

    //   // if(closestColor.color.blue > 0.8){ //&& isRedAlliance){
    //   //   missBall = true;
    //   //   System.out.println("ITS A BLUE BALL");
    //   // } else{
    //   //   missBall = false;
    //   //   System.out.println("R: " + closestColor.color.red+ "G: " + closestColor.color.green+ "B: " + closestColor.color.blue);
    //   // }
    // }

    limelight.setForceOff(!(controls.getAimShooter() || controls.getConstantAiming()) || ntDemoMode.getBoolean(false));

    double dist = limelight.getDistance();


    if(ntDemoMode.getBoolean(false)){ // Demo Controls
      double[] demoTrajectory = new double[2]; // [angle, rpm]

      if(controls.shootClose()){
        demoTrajectory[0] = 0.2;
        demoTrajectory[1] = 2000;

      } else if(controls.shootMid()){
        demoTrajectory[0] = 0;
        demoTrajectory[1] = 3100;

      } else if(controls.shootFar()){
        demoTrajectory[0] = -0.1;
        demoTrajectory[1] = 5500;

      } else if(controls.shootUp()){
        demoTrajectory[0] = -1;
        demoTrajectory[1] = 4000;

      }else{
        demoTrajectory[1] = 0;
      }
      
      shooter.setHoodPos(demoTrajectory[0]);
      shooter.setShooterRpm(demoTrajectory[1]);


      double turrTarget = controls.shooterPanLeft() ? -10 
                        : controls.shooterPanRight() ? 10
                        : 0;
        
        if((shooter.getTurretPosDegrees() > 85 && turrTarget > 0) || (shooter.getTurretPosDegrees() < -85 && turrTarget < 0))
          turrTarget = 0;

        else if (controls.overrideTurretCenter()) shooter.setTurretPos(0); // override controlled turret pos
        else shooter.setTurretDeltaPos(turrTarget); // limelight controlled turret pos;
      
    } else{ // Regular Controls

      if(controls.getAimShooter() || controls.getConstantAiming()) {
        
        if (controls.getAimShooter()) {
          TeleOpBaseRPMBuff = ntTeleopBuff.getDouble(0.0);

          rpmBuff = isAuto ? AutoBaseRPMBuff : TeleOpBaseRPMBuff;

          if(controls.getHotRPMAddition()){
            rpmBuff += hotRPMAddition.getDouble(0.0);
          } else if(controls.getHotRPMReduction()) {
            rpmBuff += -hotRPMAddition.getDouble(0.0);          
          }

          if(missBall || controls.getManualMiss()) {
            rpmBuff += ballMissRPM;
          }

          shooter.setShooterRpm( getShooterRpm(dist) + rpmBuff );
        }
        else{
          shooter.setShooterRpm(IDLE_RPM);
        }

        targetHoodPos = debouncer.calculate(limelight.hasTarget()) ? limelight.getVertAngle() : 0.0;
        
        if(targetHoodPos >= -1) {
          shooter.setHoodPos(targetHoodPos);// - (controls.getHotHoodChange() ? hotHoodAddition.getDouble(0.0) : 0.0) );
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
        
        else if (controls.overrideTurretCenter()) shooter.setTurretPos(0); // override controlled turret pos
        else shooter.setTurretDeltaPos(turrTarget); // limelight controlled turret pos;

        // ntTestHood.setDouble(targetHoodPos);
      }else{
        // shooter.setTurretPos(0);
        shooter.setShooterRpm(0);
      }
    }

    if(targetBallCount != -1 && shooter.getBallShotCount() >= targetBallCount) doneShootingFrames++;

    
    if (ntTableClimb.getEntry("Left Climb Pos").getDouble(0.0) > 8.0 && ntTableClimb.getEntry("Right Climb Pos").getDouble(0.0) > 8.0) {
      shooter.setTurretPos(90.0); // turret is at 90 degrees when both climb arms are extended
    }

  }

  public double getShooterRpm(double distance) {
    if(ntUseCalibrationMap.getBoolean(false)){
      ShootCalibrationMap.Trajectory traj;
      try {
        // traj = Constants.ShooterCalibrations.SHOOT_CALIBRATION_MAP.get(distance);
        traj = Constants.ShooterCalibrations.SHOOT_CALIBRATION_MAP.getWithRpmAdjustment(distance, rpmBuff, rpmBuffZeta);
        
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


  //method that supplies newcalibrationmap to networktables
  // public void saveNewCalibrationMap(){
  //   ntTable.getEntry("NEW Shoot Calibration Map").setString(ShooterCalibrations.NEW_SHOOT_CALIBRATION_MAP.toString());
  // }

}
