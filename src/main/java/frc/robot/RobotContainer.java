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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.autonomous.AutonomousController;
import frc.robot.commands.autonomous.InplaceTurn;
import frc.robot.commands.autonomous.paths.FiveBallCommandGroup;
import frc.robot.commands.autonomous.paths.FourBallCommandGroup;
import frc.robot.commands.autonomous.paths.OneBallCommandGroup;
import frc.robot.commands.autonomous.paths.ThreeBallCommandGroup;
import frc.robot.commands.autonomous.paths.TwoBallCommandGroup;
import frc.robot.controls.manualdrive.ManualControls;
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
  private final DriveCommand manualDrive;

  private final Limelight limelight;

  private final Shooter shooter = new Shooter();
  private final ShooterCommand manualShooter;

  private final Intake intake = new Intake();
  private final IntakeCommand manualIntake;

  private final Conveyor conveyor = new Conveyor();
  private final ConveyorCommand manualConveyor;

  private final Climb climb = new Climb();
  private final ClimbCommand manualClimb;

  private final Joystick xbox = new Joystick(0);
  private final Joystick controlStation = new Joystick(1);
  private final Joystick xbox2 = new Joystick(2);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

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

  private final OneBallCommandGroup oneBallCommandGroup;
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

    manualDrive = new DriveCommand(driveTrain, controls);
    driveTrain.setDefaultCommand(manualDrive);

    limelight = new Limelight(controls);

    manualShooter = new ShooterCommand(shooter, controls, limelight);
    shooter.setDefaultCommand(manualShooter);

    manualIntake = new IntakeCommand(intake, controls);
    intake.setDefaultCommand(manualIntake);

    manualConveyor = new ConveyorCommand(conveyor, controls, shooter);
    conveyor.setDefaultCommand(manualConveyor);

    manualClimb = new ClimbCommand(climb, controls);
    climb.setDefaultCommand(manualClimb);

    oneBallCommandGroup =  new OneBallCommandGroup(driveTrain, intake, conveyor, shooter, limelight);
    twoBallCommandGroup = new TwoBallCommandGroup(driveTrain, intake, conveyor, shooter, limelight);
    threeBallCommandGroup = new ThreeBallCommandGroup(driveTrain, intake, conveyor, shooter, limelight);
    fourBallCommandGroup = new FourBallCommandGroup(driveTrain, intake, conveyor, shooter, limelight);
    fiveBallCommandGroup = new FiveBallCommandGroup(driveTrain, intake, conveyor, shooter, limelight);

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Four Ball", fourBallCommandGroup.getCommand());

    m_chooser.addOption("One Ball", oneBallCommandGroup.getCommand());
    m_chooser.addOption("Two Ball", twoBallCommandGroup.getCommand());
    m_chooser.addOption("Three Ball", threeBallCommandGroup.getCommand());

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Auto Choose").add(m_chooser);
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

    return m_chooser.getSelected();  
  
  }
}