// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;


public class AutonomousIntake extends CommandBase {
  /** Creates a new AtonomousIntake. */

  private Intake intake;
  private Conveyor conveyor;
  public AutonomousIntake(Intake intake, Conveyor conveyor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.conveyor = conveyor;

    addRequirements(intake, conveyor);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
