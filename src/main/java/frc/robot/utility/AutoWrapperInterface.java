// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.commands.RamsexyCommand;

/** Add your docs here. */
public interface AutoWrapperInterface {
    public Trajectory getTrajectory(); 
    public RamsexyCommand getCommand();
}
