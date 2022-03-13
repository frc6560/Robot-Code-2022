// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.border.EtchedBorder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Climb;
import frc.robot.utility.NetworkTable.NtValueDisplay;

public class ClimbCommand extends CommandBase {
  /** Creates a new ManualClimb. */

  public static interface Controls {
    boolean getClimbLockEngaged();
    boolean getClimbRotatorEngaged();
    double getClimbExtensionMotors();
    
    double leftOverrideExtensionVelocity();
    double rightOverrideExtensionVelocity();
  }


  private final Climb climb;
  private final Controls controls;

  private NetworkTable nTable;
  private NetworkTableEntry extensionSpeed;
  private NetworkTableEntry rotationSpeed;
  

  public ClimbCommand(Climb climb, Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climb = climb;
    this.controls = controls;
    addRequirements(climb);

    nTable = NetworkTableInstance.getDefault().getTable("Climb");

    extensionSpeed = nTable.getEntry("Extension Speed");
    extensionSpeed.setDouble(0.8);

    rotationSpeed = nTable.getEntry("Rotation Speed");
    rotationSpeed.setDouble(0.3);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climb.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // climb.runRotatorMotor(controls.getClimbRotation() * rotationSpeed.getDouble(0.0));
    climb.setRotatorPiston(controls.getClimbRotatorEngaged());

    climb.setExtensionMotor(controls.getClimbExtensionMotors() * extensionSpeed.getDouble(0.0));
    climb.setPiston(controls.getClimbLockEngaged());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.initialize();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
