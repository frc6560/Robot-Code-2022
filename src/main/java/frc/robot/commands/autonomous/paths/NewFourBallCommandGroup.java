// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.paths;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.autonomous.InplaceTurn;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.utility.AutoWrapper;
import frc.robot.utility.StraightRamseteGen;

/** Add your docs here. */
public class NewFourBallCommandGroup implements CommandGroupInterface {

        private final DriveTrain driveTrain;
        private final Intake intake;
        private final Conveyor conveyor;
        private final Shooter shooter;
        private final Limelight limelight;

        private AutoWrapper newFourBall1;
        private AutoWrapper newFourBall2;
        private StraightRamseteGen strightRamseteGen;

        public NewFourBallCommandGroup(DriveTrain driveTrain, Intake intake, Conveyor conveyor, Shooter shooter, Limelight limelight) {
                this.driveTrain = driveTrain;
                this.intake = intake;
                this.conveyor = conveyor;
                this.shooter = shooter;
                this.limelight = limelight;

                newFourBall1 = new AutoWrapper("NewFourBall_1", driveTrain);
                newFourBall2 = new AutoWrapper("NewFourBall_2", driveTrain);
                strightRamseteGen = new StraightRamseteGen(driveTrain, -3);

        }

        @Override
        public Command getCommand() {
                return (
                        newFourBall1.getCommand().raceWith(new IntakeCommand(intake)).raceWith(new ConveyorCommand(conveyor, shooter, false)))

                        .andThen((new ShooterCommand(shooter, limelight, true, 2)).raceWith(new ConveyorCommand(conveyor, shooter, true))).withTimeout(5)

                        .andThen(newFourBall2.getCommand().raceWith(new IntakeCommand(intake)).raceWith(new ConveyorCommand(conveyor, shooter, false)))

                        .andThen(strightRamseteGen.getCommand().raceWith(new IntakeCommand(intake)).raceWith(new ConveyorCommand(conveyor, shooter, false)))        

                        .andThen((new ShooterCommand(shooter, limelight, true, 2).raceWith(new ConveyorCommand(conveyor, shooter, true)))
                        
                        );
                
        }

}
