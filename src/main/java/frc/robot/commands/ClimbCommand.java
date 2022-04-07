// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.autonomous.AutoClimbCommand;
import frc.robot.subsystems.Climb;

public class ClimbCommand extends CommandBase {
  /** Creates a new ManualClimb. */

  public static interface Controls {
    boolean getClimbRotatorEngaged();
    double getClimbExtensionMotors();
    boolean getAutoClimbEnabled();
  }


  private final Climb climb;
  private final Controls controls;
  private final AutoClimbCommand autoClimbCommand;

  public ClimbCommand(Climb climb, Controls controls) {
    this(climb, controls, null);
  }

  public ClimbCommand(Climb climb, Controls controls, AutoClimbCommand autoClimbCommand){
    this.climb = climb;
    this.controls = controls;
    addRequirements(climb);
    this.autoClimbCommand = autoClimbCommand;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climb.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(autoClimbCommand != null && controls.getClimbExtensionMotors() == 0 && controls.getAutoClimbEnabled()){
      autoClimbCommand.execute();
    }else{
      climb.setRotatorPiston(controls.getClimbRotatorEngaged());
      climb.setExtensionMotors(controls.getClimbExtensionMotors()); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
