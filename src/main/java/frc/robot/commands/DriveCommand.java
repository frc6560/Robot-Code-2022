// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  
  public interface Controls {
    double driveGetX();
    double driveGetY();
    double driveGetRotation();
  }

  private final DriveTrain drivetrain;
  private final Controls controls;
  

  public DriveCommand(DriveTrain drivetrain, Controls controls) {
    this.drivetrain = drivetrain;
    this.controls = controls;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(controls.driveGetX(), controls.driveGetY(), controls.driveGetRotation(), true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
