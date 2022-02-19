// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.RobotIds;

public class Conveyor extends SubsystemBase {
  private final CANSparkMax conveyorMotorTop = new CANSparkMax(RobotIds.CONVEYOR_MOTOR_TOP, MotorType.kBrushless);
  private final CANSparkMax conveyorMotorBottom = new CANSparkMax(RobotIds.CONVEYOR_MOTOR_BOTTOM, MotorType.kBrushless);
  
  // private final DigitalInput conveyorSensor = new DigitalInput(RobotIds.CONVEYOR_SENSOR);

  /** Creates a new Conveyor. */
  public Conveyor() {
    conveyorMotorTop.restoreFactoryDefaults();
    conveyorMotorTop.setOpenLoopRampRate(0.1);
    
    conveyorMotorBottom.restoreFactoryDefaults();
    conveyorMotorBottom.setOpenLoopRampRate(0.1);

    conveyorMotorTop.set(0.0);
    conveyorMotorBottom.set(0.0);
  }

  public void setConveyor(double output) {
    conveyorMotorTop.set(output);
    conveyorMotorBottom.set(output);
  }

  public boolean getSensor() {
    // return(conveyorSensor.get());
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
