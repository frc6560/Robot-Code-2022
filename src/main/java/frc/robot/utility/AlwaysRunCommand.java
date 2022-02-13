// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** A RunCommand that keeps running, even when the robot is disabled.
 */
public class AlwaysRunCommand extends RunCommand {
  public AlwaysRunCommand(Runnable runnable, Subsystem... requirements) {
    super(runnable, requirements);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
