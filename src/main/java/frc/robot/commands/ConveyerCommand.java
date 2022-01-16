// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Conveyer;

public class ConveyerCommand extends CommandBase {

  private final Conveyer conveyer;

  private int debounceTimer = 0;
  private int spacingTimer = 0;
  private Double output = 0.0;

  /** Creates a new ConveyerCommand. */
  public ConveyerCommand(Conveyer conveyer) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyer);
    this.conveyer = conveyer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyer.setConveyer(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controls.getCamera() && checkRun()) {
      output = 3.0;
    }
    else {
      output = 0.0;
    }

    if (controls.getCamera() && debounceTimer <= 3) {
      debounceTimer += 1;
    }

    if (!controls.getCamera() && debounceTimer > 0) {
      spacingTimer += 1;
    }

    if (spacingTimer == 3) {
      spacingTimer = 0;
      debounceTimer = 0;
    }

    conveyer.setConveyer(output);
  }

  private boolean checkRun() {
    if (debounceTimer == 3) {
      return true;
    }
    return false;
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
