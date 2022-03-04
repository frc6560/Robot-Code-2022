// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.util.Converter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ManualClimb;
import frc.robot.commands.ManualConveyor;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.ManualShooter;
import frc.robot.commands.autonomous.AutonomousController;
import frc.robot.commands.autonomous.InplaceTurn;
import frc.robot.commands.autonomous.paths.FiveBallCommandGroup;
import frc.robot.commands.autonomous.paths.FourBallCommandGroup;
import frc.robot.commands.autonomous.paths.ThreeBallCommandGroup;
import frc.robot.commands.autonomous.paths.TwoBallCommandGroup;
import frc.robot.commands.controls.manualdrive.ManualControls;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.utility.AutoUtil;
import frc.robot.utility.AutoWrapper;
import frc.robot.utility.AutoWrapperPathWeaver;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveTrain driveTrain = new DriveTrain();
  private final ManualDrive manualDrive;

  private final Limelight limelight;

  private final Shooter shooter = new Shooter();
  private final ManualShooter manualShooter;

  private final Intake intake = new Intake();
  private final ManualIntake manualIntake;

  private final Conveyor conveyor = new Conveyor();
  private final ManualConveyor manualConveyor;

  private final Climb climb = new Climb();
  private final ManualClimb manualClimb;

  private final Joystick xbox = new Joystick(0);
  private final Joystick controlStation = new Joystick(1);
  private final Joystick xbox2 = new Joystick(2);

  // declare paths
  // private AutoUtil path1 = new AutoUtil("paths/output/Test1.wpilib.json",
  // driveTrain);
  // private AutoUtil path2 = new AutoUtil("paths/output/Test2.wpilib.json",
  // driveTrain);

  // private AutoUtil linCircle = new
  // AutoUtil("paths/output/Unnamed_2.wpilib.json", driveTrain);
  // private AutoWrapper Stright = new AutoWrapper("Threeball_1 Work Copy Copy",
  // driveTrain);

  // private AutoWrapper threeBall1 = new AutoWrapper("Threeball_1", driveTrain);
  // private AutoWrapper threeBall2 = new AutoWrapper("Threeball_2", driveTrain);
  // private AutoWrapper threeBall3 = new AutoWrapper("Threeball_3", driveTrain);

  // private AutoWrapperPathWeaver stirght = new
  // AutoWrapperPathWeaver("paths/output/Unnamed.wpilib.json", driveTrain);

  private ManualControls controls;
  // private AutonomousController autonomousController;

  private final TwoBallCommandGroup twoBallCommandGroup;
  private final ThreeBallCommandGroup threeBallCommandGroup;
  private final FourBallCommandGroup fourBallCommandGroup;
  private final FiveBallCommandGroup fiveBallCommandGroup;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    controls = new ManualControls(xbox, controlStation, xbox2);

    manualDrive = new ManualDrive(driveTrain, controls);
    driveTrain.setDefaultCommand(manualDrive);

    limelight = new Limelight(controls);

    manualShooter = new ManualShooter(shooter, controls, limelight);
    shooter.setDefaultCommand(manualShooter);

    manualIntake = new ManualIntake(intake, controls);
    intake.setDefaultCommand(manualIntake);

    manualConveyor = new ManualConveyor(conveyor, controls, shooter);
    conveyor.setDefaultCommand(manualConveyor);

    manualClimb = new ManualClimb(climb, controls);
    climb.setDefaultCommand(manualClimb);

    twoBallCommandGroup = new TwoBallCommandGroup(driveTrain, intake, conveyor, shooter, limelight);
    threeBallCommandGroup = new ThreeBallCommandGroup(driveTrain, intake, conveyor, shooter, limelight);
    fourBallCommandGroup = new FourBallCommandGroup(driveTrain, intake, conveyor, shooter, limelight);
    fiveBallCommandGroup = new FiveBallCommandGroup(driveTrain, intake, conveyor, shooter, limelight);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return
    // threeBall1.getCommand().raceWith(new ManualIntake(intake)).raceWith(new
    // ManualConveyor(conveyor, shooter, false)).withTimeout(2.5).andThen(
    // (new ManualShooter(shooter, limelight, true, 2).raceWith(new
    // ManualConveyor(conveyor, shooter, true)))
    // );

    // return (threeBall1.getCommand()
    // .raceWith(new ManualIntake(intake))
    // .raceWith(new ManualConveyor(conveyor, shooter, false))
    // )

    // .andThen((new ManualShooter(shooter, limelight, false, 2))
    // .raceWith(new ManualConveyor(conveyor, shooter, true))
    // )

    // .andThen(new InplaceTurn(driveTrain, 134))

    // .andThen(threeBall3.getCommand()
    // .raceWith((new ManualIntake(intake))
    // .raceWith(new ManualConveyor(conveyor, shooter, false)))
    // )

    // .andThen((new ManualShooter(shooter, limelight, false, 1)
    // .raceWith(new ManualConveyor(conveyor, shooter, true)))
    // );

    // new InplaceTurn(driveTrain, 178)).andThen(
    // threeBall3.getCommand());
    // .raceWith(new ManualIntake(intake)).raceWith(new ManualConveyor(conveyor,
    // shooter));
    // return

    // return new InplaceTurn(driveTrain, 178);

    return twoBallCommandGroup.getCommand();
  }
}