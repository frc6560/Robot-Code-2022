// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.paths;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.utility.StraightRamseteGen;

/** Add your docs here. */
public class OneBallCommandGroup implements CommandGroupInterface {

    private final Conveyor conveyor;
    private final Shooter shooter;
    private final Limelight limelight;

    private StraightRamseteGen path1;


    public OneBallCommandGroup(DriveTrain driveTrain, Conveyor conveyor, Shooter shooter, Limelight limelight){
        this.conveyor = conveyor;
        this.shooter = shooter;
        this.limelight = limelight;

        path1 = new StraightRamseteGen(driveTrain, 1.5);
    }
    @Override
    public Command getCommand() {
        return (
            
                path1.getCommand()
                
                .andThen((new ShooterCommand(shooter, limelight, true, 1)).raceWith(new ConveyorCommand(conveyor, shooter, true)))
                );
    }

}
