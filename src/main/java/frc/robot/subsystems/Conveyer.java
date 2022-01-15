// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.RobotIds;

public class Conveyer extends SubsystemBase {
  private final CANSparkMax rollerMotor = new CANSparkMax(RobotIds.ROLLER_MOTOR, MotorType.kBrushless);
  private final CANSparkMax conveyerMotor = new CANSparkMax(RobotIds.CONVEYER_MOTOR, MotorType.kBrushless);
  private final CANSparkMax conveyerMotorOverhead = new CANSparkMax(RobotIds.CONVEYER_MOTOR_OVERHEAD, MotorType.kBrushless);

  /** Creates a new Conveyer. */
  public Conveyer() {
    rollerMotor.restoreFactoryDefaults();
    conveyerMotor.restoreFactoryDefaults();
    conveyerMotorOverhead.restoreFactoryDefaults();
    rollerMotor.setOpenLoopRampRate(0.1);
    conveyerMotor.setOpenLoopRampRate(0.1);
    conveyerMotorOverhead.setOpenLoopRampRate(0.1);

  }

  public void setConveyer(double output) {
    rollerMotor.set(output);
    conveyerMotor.set(output);
    conveyerMotorOverhead.set(output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
