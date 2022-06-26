// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;

import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotPositionSender;
import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotState;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;



public class DriveCommand extends CommandBase {
  /** Creates a new ManualDrive. */

  public static interface Controls {
    double getX();
    double getY();
    double getSpeed();
    double getTurnSpeed();
  }
  

  private final DriveTrain driveTrain;

  private final Controls controls;

  // for auto
  private volatile @NotNull Trajectory currentTrajectory;
  private volatile @NotNull RamseteCommand currentRamseteCommand;

  private double autoStartTime;

  private DriveState driveState;

  public DriveCommand(DriveTrain driveTrain, Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;

    this.controls = controls;
    

    ntDispTab("Joystick")
      .add("X Joystick", controls::getX)
      .add("Y Joystick", controls::getY)
      .add("Speed", controls::getSpeed)
      .add("Turn Speed", controls::getTurnSpeed);
  }

  public enum DriveState {
    TELEOP, RAMSETE
  }

  public void setDriveState(DriveState state) {
    driveState = state;
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.stop();
  }

  public void updateTeleOp() {
    double speed = controls.getSpeed();
    double turnSpeed = controls.getTurnSpeed();

    double x = controls.getX();
    double y = controls.getY();

    if (Math.abs(x) < 0.1) {
      x = 0;
      // driveTrain.setLVelocity(0);
      // driveTrain.setRVelocity(0);
    }

    if (Math.abs(y) < 0.1) {
      y = 0;
      // driveTrain.setLVelocity(0);
      // driveTrain.setRVelocity(0);
    }

    x *= turnSpeed;

    y *= speed;
    driveTrain.setVelocity(y, x);
  }

  public void updateRamsete() {
    this.currentRamseteCommand = new RamseteCommand(
      currentTrajectory,
      driveTrain::getPose,
      new RamseteController(2, 0.7),
      Constants.PhysicalConstants.DIFFERENTIAL_DRIVE_KINEMATICS,
      driveTrain::setWheelVelocity,
      driveTrain
    );

    currentRamseteCommand.initialize();
  }

  public void setTrajectory(Trajectory trajectory) {
    this.currentTrajectory = trajectory;
    this.autoStartTime = Timer.getFPGATimestamp();
    updateRamsete();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // TODO: Do something about this
    if (DriverStation.isTeleop()) driveState = DriveState.TELEOP;
    if (DriverStation.isAutonomous()) driveState = DriveState.RAMSETE;

    
    switch (driveState) {
      case TELEOP:
        updateTeleOp();
        break;
      case RAMSETE:
        if (this.currentRamseteCommand == null) {
          if (this.currentTrajectory == null) {
            driveState = DriveState.TELEOP;
            break;
          }
          else {
            updateRamsete(); 
          }
        }

        currentRamseteCommand.execute();

        break;
    }

    

    RobotPositionSender.addRobotPosition(new RobotState(
      driveTrain.getPose(), //The position that is graphed on the GUI
      getChassisSpeeds().vxMetersPerSecond, // Only shown as you hover over the path
      getChassisSpeeds().vyMetersPerSecond, // Only shown as you hover over the path
      getChassisSpeeds().omegaRadiansPerSecond // Only shown as you hover over the path
      ));

  }

  public ChassisSpeeds getChassisSpeeds() {
    return Constants.PhysicalConstants.DIFFERENTIAL_DRIVE_KINEMATICS.toChassisSpeeds(new DifferentialDriveWheelSpeeds(driveTrain.getLVelocity(), driveTrain.getRVelocity()));
  }

  
  public double getTrajectoryElapsedTime() {
    return autoStartTime != 0 ? Timer.getFPGATimestamp() - autoStartTime : 0;
  }

  public void stopRobot() {
    driveTrain.stop();
  }

  public DriveTrain getDriveTrain() {
    return driveTrain;
  }

  public boolean isTrajectoryDone() {
    if (currentRamseteCommand == null) return false;

    return currentRamseteCommand.isFinished() || driveState != DriveState.RAMSETE;
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.getDifferentialDrive().stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
