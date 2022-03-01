// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PWM;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Util;

import static frc.robot.Constants.*;
import static frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;

public class Shooter extends SubsystemBase {
  private final double RPMAcceptableDiff = 100;
  private final double turretAcceptableDiff = 3;
  private final double hoodAcceptableDiff = 1;

  private final double turretTurnSpeed = 0.35;

  private final TalonFX shooterMotorL;
  private final TalonFX shooterMotorR;

  private final CANSparkMax turretMotor;

  private final PWM hoodServoL;
  private final PWM hoodServoR;

  private double targetRPM;
  private double targetHoodPos;
  private double targetTurretPos;

  private NetworkTable ntTable;
  private NetworkTableEntry ntShooterReady;


  /** Creates a new Shooter. */
  public Shooter() {
    targetRPM = 0;
    targetTurretPos = 0;
    targetHoodPos = 0;


    // Shooter setup
    shooterMotorL = new TalonFX(RobotIds.SHOOTER_MOTOR_LEFT);
    shooterMotorR = new TalonFX(RobotIds.SHOOTER_MOTOR_RIGHT);

    turretMotor = new CANSparkMax(RobotIds.SHOOTER_TURRET_MOTOR, MotorType.kBrushless);
    turretMotor.getEncoder().setPosition(0.0);


    shooterMotorL.configFactoryDefault();
    shooterMotorR.configFactoryDefault();

    //get PID
    shooterMotorL.config_kF(0, 0.047197957);
    shooterMotorL.config_kP(0, 0.225);
    shooterMotorL.config_kI(0, 0.00001);
    shooterMotorL.config_kD(0, 0.0);

    shooterMotorR.config_kF(0, 0.047197957);
    shooterMotorR.config_kP(0, 0.225);
    shooterMotorR.config_kI(0, 0.00001);
    shooterMotorR.config_kD(0, 0.0);

    shooterMotorL.setInverted(false);
    shooterMotorR.setInverted(true);

    // Hood setup
    hoodServoL = new PWM(RobotIds.SHOOTER_HOOD_ACTUATOR_LEFT);
    hoodServoR = new PWM(RobotIds.SHOOTER_HOOD_ACTUATOR_RIGHT);

    hoodServoL.setBounds(2.0, 1.6, 1.5, 1.4, 1.0);  // what does this do? (take out?)
    hoodServoR.setBounds(2.0, 1.6, 1.5, 1.4, 1.0);

    ntTable = NetworkTableInstance.getDefault().getTable("Shooter");

    ntShooterReady = ntTable.getEntry("Shooter Ready");
    ntShooterReady.setBoolean(false);

    ntDispTab("Shooter")
      .add("Actual RPM", this::getShooterRpm)
      .add("Actual Turret Pos Deg", this::getTurretPosDegrees);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ntShooterReady.setBoolean(isShooterReady());

    hoodServoL.setSpeed(targetHoodPos);
    hoodServoR.setSpeed(targetHoodPos);

    shooterMotorL.set(ControlMode.Velocity, targetRPM);
    shooterMotorR.set(ControlMode.Velocity, targetRPM);
    // shooterMotorL.set(ControlMode.PercentOutput, 0.5);
    
    if(Math.abs(targetTurretPos) < turretAcceptableDiff){
      turretMotor.set(0.0);
    }else{
      double speed = Math.abs(targetTurretPos) > turretAcceptableDiff * 3  ?
                    turretTurnSpeed :
                      Math.abs(targetTurretPos) > turretAcceptableDiff * 2 ?
                    turretTurnSpeed / (3 * (3 * turretAcceptableDiff - Math.abs(targetTurretPos)) / (turretAcceptableDiff)): // basically a gradient down from 1 to 1/3
                    turretTurnSpeed / 3
      ;

      speed *= Math.copySign(1, targetTurretPos);

      turretMotor.set(speed);
    }
  }

  public void setHoodPos(double pos) {
    targetHoodPos = pos;
  }

  public void setTurretPos(double pos){
    targetTurretPos = pos;
  }

  public void setShooterRpm(double rpm) {
    targetRPM = rpm;
  }

  public double getHoodPos() {
      return targetHoodPos;
  }

  public double getShooterRpm() {
      return shooterMotorL.getSelectedSensorVelocity() * PhysicalConstants.RPM_PER_FALCON_UNIT;
  }

  public double getTurretPos(){
    return turretMotor.getEncoder().getPosition();
  }

  public double getTurretPosDegrees(){
    return getTurretPos() / 45 / (5.33333 * 1.028571428571429) * 360 + 90;
  }

  public boolean isShooterReady(){
    return 
      Math.abs(getShooterRpm()) > 200 &&
      Math.abs(getShooterRpm() - targetRPM) < RPMAcceptableDiff &&
      Math.abs(getTurretPos() - targetTurretPos) < turretAcceptableDiff &&
      Math.abs(getHoodPos() - targetHoodPos) < hoodAcceptableDiff;
  }
}
