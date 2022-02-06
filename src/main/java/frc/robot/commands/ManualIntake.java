// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Intake;

public class ManualIntake extends CommandBase {

  private final Intake intake;
  private final Controls controls;

  public static interface Controls {
    double getButtonA();
  }

  /** Creates a new IntakeCommand. */
  public ManualIntake(Intake intake, Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;
    this.controls = controls;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = 0;
    if (controls.getButtonA() == 1) {
      output = 3.0;
    }
    if (controls.getButtonA() == 0) {
      output = 0;
    }
    this.intake.setPiston(false);
    this.intake.setIntakeMotorOutput(output);
    this.intake.setHopperMotorOutput(output);
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
