// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Conveyer;

public class ConveyerCommand extends CommandBase {

  public static interface Controls {
    double ballProximityDist();
  }

  private final Conveyer conveyer;
  private final Controls controls;

  /** Creates a new ConveyerCommand2. */
  public ConveyerCommand(Conveyer conveyer, Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(conveyer);
    this.conveyer = conveyer;
    this.controls = controls;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyer.setConveyer(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controls.ballProximityDist() < 3){
      conveyer.setConveyer(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyer.setConveyer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
