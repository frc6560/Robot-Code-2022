// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.*;


public class Climb extends SubsystemBase {
  /** Creates a new Climb. */

  private final Solenoid leftPiston = new Solenoid(PneumaticsModuleType.CTREPCM, RobotIds.CLIMB_LEFT_PISTON);
  private final Solenoid rightPiston = new Solenoid(PneumaticsModuleType.CTREPCM, RobotIds.CLIMB_RIGHT_PISTON);

  private final CANSparkMax rotatorMotor = new CANSparkMax(RobotIds.CLIMB_ROTATOR_MOTOR, MotorType.kBrushless);
  private final CANSparkMax leftExtensionMotor = new CANSparkMax(RobotIds.CLIMB_LEFT_EXTENSION_MOTOR, MotorType.kBrushless);
  private final CANSparkMax rightExtensionMotor = new CANSparkMax(RobotIds.CLIMB_RIGHT_EXTENSION_MOTOR, MotorType.kBrushless);

  private final SlewRateLimiter accelLimiter = new SlewRateLimiter(3);

  private final double EPSILON = 5.08;
  private final double compConstant = 1.06;

  private double targetVelocity;

  public Climb() {
    setupAllMotors();
  }

  private void setupAllMotors() {
    setupMotor(rotatorMotor, false);
    setupMotor(leftExtensionMotor, false);
    setupMotor(rightExtensionMotor, false);
  }

  private void setupMotor(CANSparkMax motor, boolean inverted) {
    motor.restoreFactoryDefaults();
    motor.setInverted(inverted);
    motor.getEncoder().setPosition(0);
    motor.setClosedLoopRampRate(0.0);
    motor.setIdleMode(IdleMode.kBrake);
    

    // Changes default motor controller "send speed" from 20ms to 10ms
    // Add if having issues with accuracy
    //motors[0].setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double diff = checkDifference();
    
    if (diff > 0) setLeftVelocity(targetVelocity * diff * compConstant);
    if (diff < 0) setRightVelocity(targetVelocity * -diff * compConstant);
  
    if (Math.abs(diff) > EPSILON) {
      if (diff < 0) setRightPosition(getLeftPosition());
      if (diff > 0) setLeftPosition(getRightPosition());
    }
  }

  public void setTargetVelocity(double velocity) {
    targetVelocity = velocity;
  }
  

  public void setRightVelocity(double velocity) {
    rightExtensionMotor.getPIDController().setReference(velocity, ControlType.kVelocity); // rpm
  }

  public void setLeftVelocity(double velocity) {
    leftExtensionMotor.getPIDController().setReference(velocity, ControlType.kVelocity); // rpm
  }

  public void runRotatorMotor(double output) {
    rotatorMotor.set(accelLimiter.calculate(output));
  }

  public void setRotatorPosition(double pos) {
    rotatorMotor.getEncoder().setPosition(pos);
  }

  public void initialize() {
    rotatorMotor.set(0);
    leftExtensionMotor.set(0);
    rightExtensionMotor.set(0);
  }

  public void extendPistons(boolean extended) {
    leftPiston.set(extended);
    rightPiston.set(extended);
  }

  public void runLeftExtensionMotor(double output) {
    leftExtensionMotor.set(output);
  }

  public void runRightExtensionMotor(double output) {
    rightExtensionMotor.set(output);
  }

  public void runBothExtensionMotors(double output) {
    runLeftExtensionMotor(output);
    runRightExtensionMotor(output);
  }

  public double getRightExtensionMotorSpeed() {
    return rightExtensionMotor.getEncoder().getVelocity();
  }

  public double getLeftExtensionMotorSpeed() {
    return leftExtensionMotor.getEncoder().getVelocity();
  }

  public double getRightPosition() {
    return rightExtensionMotor.getEncoder().getPosition();
  }

  public double getLeftPosition() {
    return leftExtensionMotor.getEncoder().getPosition();
  }

  public void setRightPosition(double position) {
    rightExtensionMotor.getPIDController().setReference(position, ControlType.kPosition);
  }

  public void setLeftPosition(double position) {
    leftExtensionMotor.getPIDController().setReference(position, ControlType.kPosition);
  }

  public double checkDifference() {
    return getLeftPosition() - getRightPosition();
  }

}
