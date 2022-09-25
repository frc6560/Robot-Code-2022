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
import frc.robot.commands.DriveCommand;
import frc.robot.controls.manualdrive.ManualControls;
import frc.robot.subsystems.DriveTrain;
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

  @AutoBuilderAccessible
  private final DriveCommand manualDrive;

  

  private final Joystick xbox = new Joystick(0);
  private final Joystick controlStation = new Joystick(1);


  private ManualControls controls;
  // private AutonomousController autonomousController;

 

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    controls = new ManualControls(xbox, controlStation);

    manualDrive = new DriveCommand(driveTrain, controls);
    driveTrain.setDefaultCommand(manualDrive);


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