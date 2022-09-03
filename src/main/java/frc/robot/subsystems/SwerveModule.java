// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */

  private TalonFX driveMotor;
  private CANSparkMax turnMotor;

  // TODO: Change values!!
  private SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(0.001, 0.001, 0.001);
  private static final double WHEEL_RADIUS = 0.15;

  public SwerveModule(TalonFX driveMotor, CANSparkMax turnMotor) {
    this.driveMotor = driveMotor;
    this.turnMotor = turnMotor;

    // TODO: Change values!!
    this.driveMotor.config_kP(0, 0.001);
    this.driveMotor.config_kI(0, 0);
    this.driveMotor.config_kD(0, 0);

    this.turnMotor.getPIDController().setP(0.001);
    this.turnMotor.getPIDController().setI(0);
    this.turnMotor.getPIDController().setD(0);

    this.turnMotor.getEncoder().setPositionConversionFactor(2 * Math.PI);
    this.turnMotor.getPIDController().setOutputRange(-Math.PI, Math.PI);

  }
  
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotor.getSelectedSensorVelocity(), new Rotation2d(turnMotor.getEncoder().getPosition()));
  }


  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turnMotor.getEncoder().getPosition()));

    System.out.println(state.speedMetersPerSecond / WHEEL_RADIUS / (2 * Math.PI) / (10.0 / 2048.0));

    driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond / WHEEL_RADIUS / (2 * Math.PI) / (10.0 / 2048.0));
    turnMotor.getPIDController().setReference(state.angle.getRadians(), ControlType.kPosition, 0, turnFF.calculate(state.speedMetersPerSecond), ArbFFUnits.kVoltage);
  }

  

  public TalonFX getDriveMotor() {
    return this.driveMotor;
  }

  public CANSparkMax getTurnMotor() {
    return this.turnMotor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
