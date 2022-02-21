// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Climb;

public class ManualClimb extends CommandBase {
  /** Creates a new ManualClimb. */

  public static interface Controls {
    double getClimbRotation();
    boolean getClimbPiston();
    double getClimbExtensionMotors();
  }


  private final Climb climb;
  private final Controls controls;
  

  public ManualClimb(Climb climb, Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climb = climb;
    this.controls = controls;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climb.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.runRotatorMotor(controls.getClimbRotation());
    climb.setPiston(controls.getClimbPiston());
    climb.setTargetVelocity(controls.getClimbExtensionMotors());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.initialize();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
