// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
import frc.robot.utility.Util;

public class Shooter extends SubsystemBase {
  private final TalonFX shooterMotorL;
  private final TalonFX shooterMotorR;

  private final CANSparkMax turretMotor;

  private final PWM hoodServoL;
  private final PWM hoodServoR;
  private final Encoder hoodEncoder;

  private double targetRPM;
  private double targetHoodPos;
  private double targetTurretPos;

  private NetworkTable ntTable;


  /** Creates a new Shooter. */
  public Shooter() {
    targetRPM = 0;
    targetHoodPos = 0;
    targetTurretPos = 0;

    // Shooter setup
    shooterMotorL = new TalonFX(RobotIds.SHOOTER_MOTOR_LEFT);
    shooterMotorR = new TalonFX(RobotIds.SHOOTER_MOTOR_RIGHT);

    turretMotor = new CANSparkMax(RobotIds.SHOOTER_TURRET_MOTOR, MotorType.kBrushless);


    shooterMotorL.configFactoryDefault();
    shooterMotorR.configFactoryDefault();

    //get PID
    shooterMotorL.config_kF(0, 0.047197957);
    shooterMotorL.config_kP(0, 0.15);
    shooterMotorL.config_kI(0, 0.0);
    shooterMotorL.config_kD(0, 0.0);

    shooterMotorR.config_kF(0, 0.047197957);
    shooterMotorR.config_kP(0, 0.15);
    shooterMotorR.config_kI(0, 0.0);
    shooterMotorR.config_kD(0, 0.0);  

    shooterMotorL.setInverted(true);
    shooterMotorR.setInverted(false);

    // Hood setup
    hoodServoL = new PWM(1);
    hoodServoR = new PWM(2);
    hoodEncoder = new Encoder(RobotIds.SHOOTER_HOOD_ENCODER_A, RobotIds.SHOOTER_HOOD_ENCODER_B);

    hoodServoL.setBounds(2.0, 1.6, 1.5, 1.4, 1.0);
    hoodServoR.setBounds(2.0, 1.6, 1.5, 1.4, 1.0);
    hoodEncoder.reset();

    ntTable = NetworkTableInstance.getDefault().getTable("Shooter");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setHoodPos(ntTable.getEntry("Hood Position").getDouble(0.0));
    setShooterRpm(ntTable.getEntry("Shooter RPM").getDouble(0.0));

    hoodServoL.setSpeed(Util.getLimited((targetHoodPos - hoodEncoder.getDistance()) / 40.0, 1.0));
    hoodServoR.setSpeed(Util.getLimited((targetHoodPos - hoodEncoder.getDistance()) / 40.0, 1.0));

    shooterMotorL.set(ControlMode.Velocity, targetRPM / PhysicalConstants.RPM_PER_FALCON_UNIT);
    shooterMotorR.set(ControlMode.Velocity, targetRPM / PhysicalConstants.RPM_PER_FALCON_UNIT);

    if(Math.abs(getHoodPos()-targetHoodPos) > 5){
      turretMotor.set(0.3);
    }else{
      turretMotor.set(0.0);
    }
  }

  public void setHoodPos(double pos) {
    targetHoodPos = pos - PhysicalConstants.MAX_HOOD_ENCODER_DISTANCE;
  }

  public void setTurretPos(double pos){
    targetTurretPos = pos;
  }

  public void setShooterRpm(double rpm) {
    targetRPM = rpm;
  }

  public double getHoodPos() {
      return hoodEncoder.getDistance() + PhysicalConstants.MAX_HOOD_ENCODER_DISTANCE;
  }

  public double getShooterRpm() {
      return shooterMotorL.getSelectedSensorVelocity() * PhysicalConstants.RPM_PER_FALCON_UNIT;
  }

  public double getTurretPos(){
    return turretMotor.getEncoder().getPosition();
  }
}
