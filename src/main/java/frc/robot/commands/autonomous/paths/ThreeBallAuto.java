// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utility.AutoWrapper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallAuto extends SequentialCommandGroup {

  private DriveTrain driveTrain;
  private AutoWrapper path1;
  private AutoWrapper path2;
  /** Creates a new ThreeBallAuto. */
  public ThreeBallAuto(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;

    path1 = new AutoWrapper("Threeball1", driveTrain);
    // super(arg0)
  }
}
