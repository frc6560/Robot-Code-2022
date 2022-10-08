// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */

  private TalonFX driveMotor;
  private CANSparkMax turnMotor;

  // TODO: Change values!!
  private static final double WHEEL_RADIUS = 0.0508;
  private static final double TURN_EPSILON = 0.01; // rads

  public SwerveModule(TalonFX driveMotor, CANSparkMax turnMotor) {
    this.driveMotor = driveMotor;
    this.turnMotor = turnMotor;

    // TODO: Change values!!
    // driveMotor.configOpenloopRamp(1);
    driveMotor.configFactoryDefault();
    driveMotor.setNeutralMode(NeutralMode.Brake);
    driveMotor.config_kF(0, 0.05);
    driveMotor.config_kP(0, 0.1);
    driveMotor.config_kI(0, 0.0001);
    driveMotor.config_kD(0, 0.00001);

    

    turnMotor.restoreFactoryDefaults();
    turnMotor.setIdleMode(IdleMode.kBrake);
    turnMotor.getPIDController().setFF(0);
    turnMotor.getPIDController().setP(5);
    turnMotor.getPIDController().setI(0.001);
    turnMotor.getPIDController().setD(1.5);
    turnMotor.getPIDController().setOutputRange(-1, 1);
    turnMotor.getEncoder().setPositionConversionFactor(Math.pow(2 * Math.PI, 2) / 134.6);


    // this.turnMotor.getPIDController().setOutputRange(-Math.PI, Math.PI);

  }
  
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotor.getSelectedSensorVelocity(), new Rotation2d(turnMotor.getEncoder().getPosition()));
  }


  public void setDesiredState(SwerveModuleState state) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    state = SwerveModuleState.optimize(state, new Rotation2d(turnMotor.getEncoder().getPosition()));
    System.out.println(state.speedMetersPerSecond);

    driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond / WHEEL_RADIUS / (2 * Math.PI) / (10.0 / 2048.0));
    // driveMotor.set(ControlMode.Velocity, 200);//ControlMode.Velocity, state.speedMetersPerSecond / WHEEL_RADIUS / (2 * Math.PI) / (10.0 / 2048.0));
    System.out.println(driveMotor.getSelectedSensorVelocity());
    // System.out.println(state.angle.getRadians());
    // System.out.println(turnMotor.getEncoder().getPosition());

    if (Math.abs(turnMotor.getEncoder().getPosition() - state.angle.getRadians()) > TURN_EPSILON)
      turnMotor.getPIDController().setReference(state.angle.getRadians(), ControlType.kPosition);
    else turnMotor.set(0.0);
  }

  public void resetRotation() {
    turnMotor.getEncoder().setPosition(0.0);
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
