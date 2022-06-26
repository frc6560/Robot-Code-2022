// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.dacubeking.AutoBuilder.robot.annotations.AutoBuilderAccessible;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.RobotIds;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.autonomous.AutoClimbCommand;
import frc.robot.controls.manualdrive.ManualControls;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RGBLighting;
import frc.robot.subsystems.Shooter;

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
  
  public static final I2C.Port colorSensorPort = I2C.Port.kOnboard;
  public static final ColorSensorV3 colorSensor = new ColorSensorV3(colorSensorPort);
  
  public static final DigitalInput conveyorSensor = new DigitalInput(RobotIds.CONVEYOR_SENSOR);


  private final DriveTrain driveTrain = new DriveTrain();

  @AutoBuilderAccessible
  private final DriveCommand manualDrive;

  private final Limelight limelight;

  private final Shooter shooter = new Shooter();

  @AutoBuilderAccessible
  private final ShooterCommand manualShooter;

  private final Intake intake = new Intake();

  @AutoBuilderAccessible
  private final IntakeCommand manualIntake;

  private final Conveyor conveyor = new Conveyor();

  @AutoBuilderAccessible
  private final ConveyorCommand manualConveyor;

  private final Climb climb = new Climb();

  @AutoBuilderAccessible
  private final ClimbCommand manualClimb;

  private final Joystick xbox = new Joystick(0);
  private final Joystick controlStation = new Joystick(1);


  private ManualControls controls;
  // private AutonomousController autonomousController;

  private final AutoClimbCommand autoClimb = new AutoClimbCommand(climb);


  private final RGBLighting rgbLighting;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    controls = new ManualControls(xbox, controlStation);

    manualDrive = new DriveCommand(driveTrain, controls);
    driveTrain.setDefaultCommand(manualDrive);

    limelight = new Limelight(controls);

    manualShooter = new ShooterCommand(shooter, controls, limelight);
    shooter.setDefaultCommand(manualShooter);

    manualIntake = new IntakeCommand(intake, controls);
    intake.setDefaultCommand(manualIntake);

    manualConveyor = new ConveyorCommand(conveyor, controls, shooter);
    conveyor.setDefaultCommand(manualConveyor);

    manualClimb = new ClimbCommand(climb, controls, autoClimb);
    climb.setDefaultCommand(manualClimb);

    rgbLighting = new RGBLighting();

  }

  public DriveCommand getDriveCommand() {
    return this.manualDrive;
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
}