// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.paths;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.utility.AutoWrapper;

/** Add your docs here. */
public class TwoBallCommandGroup implements CommandGroupInterface {

    private final DriveTrain driveTrain;
    private final Intake intake;
    private final Conveyor conveyor;
    private final Shooter shooter;
    private final Limelight limelight;

    private AutoWrapper threeBall1;

    public TwoBallCommandGroup(DriveTrain driveTrain, Intake intake, Conveyor conveyor, Shooter shooter, Limelight limelight){
        this.driveTrain = driveTrain;
        this.intake = intake;
        this.conveyor = conveyor;
        this.shooter = shooter;
        this.limelight = limelight;

        threeBall1 = new AutoWrapper("Threeball_1", driveTrain);
    }
    @Override
    public Command getCommand() {
        return (
            
                threeBall1.getCommand().raceWith(new IntakeCommand(intake)).raceWith(new ConveyorCommand(conveyor, shooter, false)))
                
                .andThen((new ShooterCommand(shooter, limelight, false, 2)).raceWith(new ConveyorCommand(conveyor, shooter, true))    
                
                );
    }

}
