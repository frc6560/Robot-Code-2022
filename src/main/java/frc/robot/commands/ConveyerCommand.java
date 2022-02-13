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
  private double output = 0.0;

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
    if (checkRun()) {
      output = 3.0;
    }
    else {
      output = 0.0;
    }

    if (debounceTimer <= 3) { //Checks if it has been three frames to run
      debounceTimer += 1;
    }

    if (debounceTimer > 0) { //Checks if the ball is gone to increase spacing
      spacingTimer += 1;
    }

    if (spacingTimer >= 3) { //Checks if spacing is three to reset
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
  public void end(boolean interrupted) {
    conveyer.setConveyer(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
