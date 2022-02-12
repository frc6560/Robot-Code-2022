// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ManualShooter extends CommandBase {

  public static interface Controls {
    boolean isShooting();
  }

  public static interface Limelight {
    double distanceToTarget();
  }


  private Shooter shooter;
  private Controls controls;
  private Limelight limelight;

  public ManualShooter(Shooter shooter, Controls controls, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.controls = controls;
    this.limelight = limelight;

    addRequirements(shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setHoodPos(0.0);
    shooter.setShooterRpm(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controls.isShooting()) {
      shooter.setShooterRpm(getShooterRpm(limelight.distanceToTarget()));
      shooter.setHoodPos(getShooterAngle(limelight.distanceToTarget()));
    }
    else {
      shooter.setShooterRpm(0.0);
    }
  }

  public double getShooterRpm(double distance) {
    return distance; //TODO: add function
  }

  public double getShooterAngle(double distance) {
    return distance; //TODO: add function
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShooterRpm(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
