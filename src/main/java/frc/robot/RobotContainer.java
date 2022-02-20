// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ManualConveyor;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.ManualShooter;
import frc.robot.commands.controls.manualdrive.ManualControls;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
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

  private final Joystick xbox = new Joystick(0);
  private final Joystick controlStation = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    ManualControls controls = new ManualControls(xbox, controlStation);

    manualDrive = new ManualDrive(driveTrain, controls);
    driveTrain.setDefaultCommand(manualDrive);

    limelight = new Limelight(controls);

    manualShooter = new ManualShooter(shooter, controls, limelight);
    shooter.setDefaultCommand(manualShooter);

    manualIntake = new ManualIntake(intake, controls);
    intake.setDefaultCommand(manualIntake);

    manualConveyor = new ManualConveyor(conveyor, controls);
    conveyor.setDefaultCommand(manualConveyor);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}