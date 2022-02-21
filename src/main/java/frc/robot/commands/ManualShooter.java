// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.utility.Util;
import frc.robot.commands.controls.manualdrive.ManualControls;
import frc.robot.subsystems.Limelight;

public class ManualShooter extends CommandBase {

  public static interface Controls {
    boolean isShooting();
    double shooterHoodTest();
    double shooterTurretTest();
    double shooterRPMTest();
  }

  private Shooter shooter;
  private Controls controls;
  private Limelight limelight;
  private final double maxShooterSpeed = 5000;

  private NetworkTable ntTable;
  private NetworkTableEntry ntTestRPM;
  private NetworkTableEntry ntTestHood;

  private double targetHoodPos = 0.0;

  public ManualShooter(Shooter shooter, Controls controls, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.controls = controls;
    this.limelight = limelight;

    addRequirements(shooter);

    
    this.ntTable = NetworkTableInstance.getDefault().getTable("Shooter");

    ntTestHood = ntTable.getEntry("Hood Angle");
    ntTestHood.setDouble(-1.0);

    ntTestRPM = ntTable.getEntry("Target RPM");
    ntTestRPM.setDouble(0.0);

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
    if (true) {
      targetHoodPos += controls.shooterHoodTest()/100;
      targetHoodPos = Util.getLimited(targetHoodPos, 1);

      ntTestHood.setDouble(targetHoodPos);

      shooter.setShooterRpm(getShooterRpm(limelight.getDistance()) * controls.shooterRPMTest());
      shooter.setHoodPos(targetHoodPos);
      // shooter.setTurretPos(limelight.getAngle());

      // shooter.setHoodPos(shooter.getHoodPos() + controls.shooterHoodTest() * 0.5);
      shooter.setTurretPos(shooter.getTurretPos() + controls.shooterTurretTest());
      // shooter.setShooterRpm(controls.shooterRPMTest() * maxShooterSpeed);
    }
    else {
      shooter.setShooterRpm(0.0);
    }

  }

  public double getShooterRpm(double distance) {
    return ntTestRPM.getDouble(0.0) * 3.454545457;
    // return distance; //TODO: add function
  }

  public double getShooterAngle(double distance) {
    return ntTestHood.getDouble(0.0);
    // return distance; //TODO: add function
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
