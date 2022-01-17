// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ManualClimb;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.controls.manualdrive.ManualControls;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private DriveTrain driveTrain = new DriveTrain();
  private ManualDrive manualDrive;

  private Climb climb = new Climb();
  private ManualClimb manualClimb;
  

  private Joystick xbox = new Joystick(0);
  private Joystick controlStation = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    ManualControls controls = new ManualControls(xbox, controlStation);

    manualDrive = new ManualDrive(driveTrain, controls);
    driveTrain.setDefaultCommand(manualDrive);

    manualClimb = new ManualClimb(climb, controls);
    climb.setDefaultCommand(manualClimb);
    

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