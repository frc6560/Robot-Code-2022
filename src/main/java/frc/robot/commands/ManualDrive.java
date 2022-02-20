// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;



public class ManualDrive extends CommandBase {
  /** Creates a new ManualDrive. */

  public static interface Controls {
    double getX();
    double getY();
    double getSpeed();
    double getTurnSpeed();
  }
  

  private final DriveTrain driveTrain;

  private final Controls controls;

  public ManualDrive(DriveTrain driveTrain, Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;

    this.controls = controls;
    

    ntDispTab("Joystick")
      .add("X Joystick", () -> controls.getX())
      .add("Y Joystick", () -> controls.getY())
      .add("Speed", () -> controls.getSpeed())
      .add("Turn Speed", () -> controls.getTurnSpeed());

    
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.getDifferentialDrive().stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(driveTrain.getCurrentPose().getRotation());
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

    //System.out.println(driveTrain.getCurrentPose());
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
