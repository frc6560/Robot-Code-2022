// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.RobotIds;
import frc.robot.utility.NetworkTable.NtValueDisplay;

public class Climb extends SubsystemBase {

  private final Solenoid rotatorPiston = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

  private final TalonFX leftExtensionMotor = new TalonFX(RobotIds.CLIMB_LEFT_EXTENSION_MOTOR);
  private final TalonFX rightExtensionMotor = new TalonFX(RobotIds.CLIMB_RIGHT_EXTENSION_MOTOR);

  private final double minPos = 0;
  private final double maxPos = 22.5;

  private double rightComp = 1;
  private double leftComp = 1;
  private final double comp_beta = 1.05;

  private NetworkTable nTable;
  private NetworkTableEntry ntOverideSoftLimit;
  private NetworkTableEntry rightCompensationConstant;
  private NetworkTableEntry ntExtensionSpeed;
  private NetworkTableEntry rightTestSpeed;
  private NetworkTableEntry leftTestSpeed;

  /** Creates a new Climb. */
  public Climb() {
    setupAllMotors();
    nTable = NetworkTableInstance.getDefault().getTable("Climb");

    ntExtensionSpeed = nTable.getEntry("Extension Speed");
    ntExtensionSpeed.setDouble(0.9);

    ntOverideSoftLimit = nTable.getEntry("Climb Override");
    ntOverideSoftLimit.setBoolean(false);
    
    rightTestSpeed = nTable.getEntry("Right Override speed");
    rightTestSpeed.setDouble(0.0);

    leftTestSpeed = nTable.getEntry("left Override speed");
    leftTestSpeed.setDouble(0.0);
    

    NtValueDisplay.ntDispTab("Climb")
        .add("Left Pos", this::getLeftPositionInches)
        .add("Right Pos", this::getRightPositionInches);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void setRotatorPiston(boolean extended) {
    rotatorPiston.set(extended);
  }

  public void reset() {
    leftExtensionMotor.set(TalonFXControlMode.PercentOutput, 0);
    rightExtensionMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void setExtensionMotors(double output) {
    double rightPos = getRightPositionInches();
    double leftPos = getLeftPositionInches();

    if (Math.abs(output) != 1) {
      output = 0;
    }

    double extensionSpeed = ntExtensionSpeed.getDouble(0.5);

    double leftSpeed;
    double rightSpeed;
    
    if(!ntOverideSoftLimit.getBoolean(false)){

      if(output > 0){
        if (leftPos < maxPos) {
          leftSpeed = (output * extensionSpeed);
        }else{
          leftSpeed = (0.0);
        }
    
        if (rightPos < maxPos) {
          rightSpeed = (output * extensionSpeed);
        }else{
          rightSpeed = (0.0);
        }
      }else{
        output /= 1.5;

        if (leftPos > minPos) {
          leftSpeed = (output * extensionSpeed);
        }else{
          leftSpeed = (0.0);
        }
    
        if (rightPos > minPos) {
          rightSpeed = (output * extensionSpeed);
        }else{
          rightSpeed = (0.0);
        }
      }
    }else{
      leftSpeed = (output * extensionSpeed);
      rightSpeed = (output * extensionSpeed);
    }

    double diff = getRightPositionInches() - getLeftPositionInches();
    // double dir = Math.copySign(1, output);

    if(output > 0){
      rightComp = 1;
      leftComp = Math.min(2, Math.max(1 + diff/0.5, 0));
    } else{
      leftComp = 1;
      rightComp = Math.min(2, Math.max(1 + diff/0.5, 0));
    }



    // if(diff > 0){
      // leftComp = Math.min(2, Math.max(1 + diff/100, 0));
    //   rightComp = 1.0;
    // } else{
      // rightComp = 1 + diff/10;
    //   leftComp = 1.0;
    // }


    setLeftExtensionMotor(leftSpeed * leftComp);
    setRightExtensionMotor(rightSpeed * rightComp);
  }

  private void setLeftExtensionMotor(double output) {
    leftExtensionMotor.set(TalonFXControlMode.PercentOutput, output);
  }

  private void setRightExtensionMotor(double output) {
    rightExtensionMotor.set(TalonFXControlMode.PercentOutput, output);
  }

  private void setupAllMotors() {
    setupMotor(leftExtensionMotor, true);
    setupMotor(rightExtensionMotor, true);
  }

  private void setupMotor(TalonFX motor, boolean inverted) {
    motor.configFactoryDefault();
    motor.setInverted(inverted);
    motor.setSelectedSensorPosition(0.0);
    motor.setNeutralMode(NeutralMode.Brake);

    // motor.configForwardSoftLimitThreshold(100);
    // motor.configForwardSoftLimitEnable(true);
  }

  public double getRightPosition() {
    return rightExtensionMotor.getSelectedSensorPosition();
  }

  public double getLeftPosition() {
    return leftExtensionMotor.getSelectedSensorPosition();
  }

  public double getRightPositionInches() {
    return getRightPosition() / 2048 * PhysicalConstants.CLIMB_EXTENSION_INCHES_PER_ROTATION;
  }

  public double getLeftPositionInches() {
    return getLeftPosition() / 2048 * PhysicalConstants.CLIMB_EXTENSION_INCHES_PER_ROTATION;
  }

  public double getRightVelocity() {
    return rightExtensionMotor.getSelectedSensorVelocity();
  }

  public double getleftVelocity() {
    return leftExtensionMotor.getSelectedSensorVelocity();
  }

  // public double getRightVelocityInches() {
  //   return getRightVelocity() * PhysicalConstants.CLIMB_EXTENSION_INCHES_PER_ROTATION;
  // }

  // public double getLeftVelocityInches() {
  //   return getRightVelocity() * PhysicalConstants.CLIMB_EXTENSION_INCHES_PER_ROTATION;
  // }

}
