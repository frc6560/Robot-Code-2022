// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  private final Solenoid lockingPiston = new Solenoid(PneumaticsModuleType.CTREPCM, RobotIds.CLIMB_PISTON);
  private final Solenoid rotatorPiston = new Solenoid(PneumaticsModuleType.CTREPCM, RobotIds.CLIMB_ROTATOR_PISTON);

  private final TalonFX leftExtensionMotor = new TalonFX(RobotIds.CLIMB_LEFT_EXTENSION_MOTOR);
  private final TalonFX rightExtensionMotor = new TalonFX(RobotIds.CLIMB_RIGHT_EXTENSION_MOTOR);

  private final double minPos = 0;
  private final double maxPosRetracted = 17000; // TODO: change
  private final double maxPosExtended = 17650;

  private NetworkTable nTable;
  private NetworkTableEntry ntOverideSoftLimit;
  private NetworkTableEntry rightCompensationConstant;
  private NetworkTableEntry ntExtensionSpeed;

  /** Creates a new Climb. */
  public Climb() {
    setupAllMotors();
    nTable = NetworkTableInstance.getDefault().getTable("Climb");

    rightCompensationConstant = nTable.getEntry("Right Compensation Constant");
    rightCompensationConstant.setDouble(0.99);

    ntExtensionSpeed = nTable.getEntry("Extension Speed");
    ntExtensionSpeed.setDouble(0.5);

    ntOverideSoftLimit = nTable.getEntry("Climb Override");
    ntOverideSoftLimit.setBoolean(false);

    NtValueDisplay.ntDispTab("Climb").add("Left Pos", this::getLeftPositionInches).add("Right Pos",
        this::getRightPositionInches);
    NtValueDisplay.ntDispTab("Climb").add("Left Vel", this::getLeftVelocityInches).add("Right Vel",
        this::getRightVelocityInches);

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

    // System.out.println(output);
    double extensionSpeed = ntExtensionSpeed.getDouble(0.5);
    
    if(ntOverideSoftLimit.getBoolean(false) == false){
      if(output > 0){
        if (leftPos < maxPosExtended) {
          setLeftExtensionMotor(output * extensionSpeed);
        }else{
          setLeftExtensionMotor(0.0);
        }
    
        if (rightPos < maxPosExtended) {
          setRightExtensionMotor(output * extensionSpeed);
        }else{
          setRightExtensionMotor(0.0);
        }
      }else{
        if (leftPos > minPos) {
          setLeftExtensionMotor(output * extensionSpeed);
        }else{
          setLeftExtensionMotor(0.0);
        }
    
        if (rightPos > minPos) {
          setRightExtensionMotor(output * extensionSpeed);
        }else{
          setRightExtensionMotor(0.0);
        }
      }
    }else{
      setLeftExtensionMotor(output * extensionSpeed);
      setRightExtensionMotor(output * extensionSpeed);
    }
    

  }

  private void setLeftExtensionMotor(double output) {
    leftExtensionMotor.set(TalonFXControlMode.PercentOutput, output);
  }

  private void setRightExtensionMotor(double output) {
    System.out.println(output * rightCompensationConstant.getDouble(0.99));

    rightExtensionMotor.set(TalonFXControlMode.PercentOutput, output * rightCompensationConstant.getDouble(1.0));
  }

  private void setupAllMotors() {
    setupMotor(leftExtensionMotor, true);
    setupMotor(rightExtensionMotor, true);
  }

  private void setupMotor(TalonFX motor, boolean inverted) {
    motor.configFactoryDefault();
    motor.setInverted(inverted);
    motor.setSelectedSensorPosition(0.0);
  }

  public double getRightPosition() {
    return rightExtensionMotor.getSelectedSensorPosition();
  }

  public double getLeftPosition() {
    return leftExtensionMotor.getSelectedSensorPosition();
  }

  public double getRightVelocity() {
    return rightExtensionMotor.getSelectedSensorVelocity();
  }

  public double getleftVelocity() {
    return leftExtensionMotor.getSelectedSensorVelocity();
  }

  public double getRightPositionInches() {
    return getRightPosition() * PhysicalConstants.CLIMB_EXTENSION_INCHES_PER_ROTATION;
  }

  public double getLeftPositionInches() {
    return getLeftPosition() * PhysicalConstants.CLIMB_EXTENSION_INCHES_PER_ROTATION;
  }

  public double getRightVelocityInches() {
    return getRightVelocity() * PhysicalConstants.CLIMB_EXTENSION_INCHES_PER_ROTATION;
  }

  public double getLeftVelocityInches() {
    return getRightVelocity() * PhysicalConstants.CLIMB_EXTENSION_INCHES_PER_ROTATION;
  }

  public void setPiston(boolean extended) {
    lockingPiston.set(extended);
  }
}
