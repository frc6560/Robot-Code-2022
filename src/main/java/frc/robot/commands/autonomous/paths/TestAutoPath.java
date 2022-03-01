// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.paths;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainLeoGood;
// import frc.robot.subsystems.DriveTrain;
import frc.robot.utility.AutoUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAutoPath extends SequentialCommandGroup {
  /** Creates a new TestAutoPath. */
  AutoUtil path1;
  AutoUtil path2;
  
  public TestAutoPath(DriveTrainLeoGood driveTrain) {
    path1 = new AutoUtil("paths/output/straight.wpilib.json", driveTrain);
    path2 = new AutoUtil("paths/output/right.wpilib.json", driveTrain);
    
    addCommands(
      path1.getCommand(), 
      path2.getCommand());
  }
}
