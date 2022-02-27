// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.lang.invoke.ConstantBootstraps;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.utility.ShootCalibrationMap;
import frc.robot.utility.Util;
import frc.robot.Constants;
import frc.robot.Constants.ShooterCalibrations;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Limelight;

public class AutonomousShooter extends CommandBase {

  private Shooter shooter;
  private Conveyor conveyor;
  private Limelight limelight;

  private NetworkTable ntTable;
  private NetworkTableEntry ntTestRPM;
  private NetworkTableEntry ntTestHood;


  private double targetHoodPos = 0.0;
  private double targetTurretPos = 0.0;
  private double targetShooterRpm = 0.0;

  public AutonomousShooter(Shooter shooter, Conveyor conveyor, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.limelight = limelight;
    this.conveyor = conveyor;

    addRequirements(shooter, limelight, conveyor);

    
    this.ntTable = NetworkTableInstance.getDefault().getTable("Shooter");

    ntTestHood = ntTable.getEntry("Hood Angle");
    ntTestHood.setDouble(0.0);

    ntTestRPM = ntTable.getEntry("Target RPM");
    ntTestRPM.setDouble(0.0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setHoodPos(0.0);
    shooter.setShooterRpm(0.0);
    shooter.setTurretPos(0); // TODO: safety... what is 0?
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limelight.setForceOff(false);
    double dist = limelight.getDistance();

    targetHoodPos = getShooterHoodAngle(dist);
    targetTurretPos = limelight.getHorizontalAngle();
    targetShooterRpm = getShooterRpm(dist) * 3.454545457;

    shooter.setShooterRpm(targetShooterRpm);
    if(targetHoodPos >= -1){
      shooter.setHoodPos(targetHoodPos);
    }
    shooter.setTurretPos(targetTurretPos); // limelight controlled turret pos;


    if (shooter.isShooterReady()) { // TODO: Change shooter frame constant
      conveyor.setConveyor(0.45);
      conveyor.setOverHead(0.7);
    }
    
    // ntTestHood.setDouble(targetHoodPos);
  }

  public double getShooterRpm(double distance) {
    ShootCalibrationMap.Trajectory traj;
      try {
        traj = Constants.ShooterCalibrations.SHOOT_CALIBRATION_MAP.get(distance);
        
      } catch (ShootCalibrationMap.OutOfBoundsException e) {
        return 0.0;
      }

      return traj.shooterRpm;
    // return distance; //TODO: add function
  }

  public double getShooterHoodAngle(double distance) {
    ShootCalibrationMap.Trajectory traj;
      try {
        traj = Constants.ShooterCalibrations.SHOOT_CALIBRATION_MAP.get(distance);
        
      } catch (ShootCalibrationMap.OutOfBoundsException e) {
        return 0.0;
      }

      return traj.hoodPos;
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShooterRpm(0.0);
    shooter.setTurretPos(0.0);
    conveyor.setOverHead(0.0);
    conveyor.setConveyor(0.0);
    limelight.setForceOff(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
