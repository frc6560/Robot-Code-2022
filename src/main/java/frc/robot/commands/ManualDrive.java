// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;



public class ManualDrive extends CommandBase {
  /** Creates a new ManualDrive. */

  public static interface Controls {
    double getX();
    double getY();
    double getSpeed();
  }
  

  private final DriveTrain driveTrain;
  private final DifferentialDrive drive;

  private final Controls controls;

  public ManualDrive(DriveTrain driveTrain, Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.drive = driveTrain.drive;

    this.controls = controls;
    

    ntDispTab("Joystick")
      .add("X Joystick", () -> controls.getX())
      .add("Y Joystick", () -> controls.getY());

    
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.stopMotor();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controls.getSpeed();
    double x = controls.getX() * speed;
    double y = controls.getY() * speed;

    if (Math.abs(x) < 0.1) {
      x = 0;
    }

    if (Math.abs(y) < 0.1) {
      y = 0;
    }

    drive.arcadeDrive(-y, x, true);



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
