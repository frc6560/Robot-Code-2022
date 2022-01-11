// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants.*;

public class ShooterVoltageTest extends SubsystemBase {
  /** Creates a new ShooterVoltageTest. */

  private final TalonFX leftShooterMotor = new TalonFX(RobotIds.SHOOTER_MOTOR_LEFT);
  private final TalonFX rightShooterMotor = new TalonFX(RobotIds.SHOOTER_MOTOR_RIGHT);

  private NetworkTableEntry leftMotorTableEntry;
  private NetworkTableEntry rightMotorTableEntry;
  private NetworkTableEntry leftMotorRPM;
  private NetworkTableEntry rightMotorRPM;
  private NetworkTableEntry motorTableEntry;

  public ShooterVoltageTest() {
    leftShooterMotor.configFactoryDefault();
    rightShooterMotor.configFactoryDefault();
    
    leftShooterMotor.configOpenloopRamp(1);
    rightShooterMotor.configOpenloopRamp(1);

    rightShooterMotor.setInverted(false);
    leftShooterMotor.setInverted(true);

    // TODO: Calculate optimal PID constants
    // leftShooterMotor.config_kF(0, 0.047197957);
    // leftShooterMotor.config_kP(0, 0.15);
    // leftShooterMotor.config_kI(0, 0.0);
    // leftShooterMotor.config_kD(0, 0.0);

    // rightShooterMotor.config_kF(0, 0.047197957);
    // rightShooterMotor.config_kP(0, 0.15);
    // rightShooterMotor.config_kI(0, 0.0);
    // rightShooterMotor.config_kD(0, 0.0);


    final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Shooter");
    //leftMotorTableEntry = ntTable.getEntry("Left Motor % Output");
    //rightMotorTableEntry = ntTable.getEntry("Right Motor % Output");
    //leftMotorTableEntry.setDouble(0.0);
    //rightMotorTableEntry.setDouble(0.0);

    motorTableEntry = ntTable.getEntry("Motor Approx Target RPM");
    motorTableEntry.setDouble(0.0);
    

    leftMotorRPM = ntTable.getEntry("Left Motor RPM");
    rightMotorRPM = ntTable.getEntry("Right Motor RPM");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //double leftOutput = leftMotorTableEntry.getDouble(0) / 100.0;
    //double rightOutput = rightMotorTableEntry.getDouble(0) / 100.0;

    double output = motorTableEntry.getDouble(0.0) / 6200.0 * (1+ (motorTableEntry.getDouble(0.0) - leftMotorRPM.getDouble(0.0))/6200);
    //double output = motorTableEntry.getDouble(0.0) / PhysicalConstants.RPM_PER_FALCON_UNIT;

    leftShooterMotor.set(ControlMode.PercentOutput, output);
    rightShooterMotor.set(ControlMode.PercentOutput, output);

    leftMotorRPM.setDouble(leftShooterMotor.getSelectedSensorVelocity() * PhysicalConstants.RPM_PER_FALCON_UNIT);
    rightMotorRPM.setDouble(rightShooterMotor.getSelectedSensorVelocity() * PhysicalConstants.RPM_PER_FALCON_UNIT);
  
  }
}
