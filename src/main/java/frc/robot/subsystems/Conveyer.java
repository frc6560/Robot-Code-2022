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
  private final CANSparkMax conveyerMotorTop = new CANSparkMax(RobotIds.CONVEYER_MOTOR_TOP, MotorType.kBrushless);
  private final CANSparkMax conveyerMotorBottom = new CANSparkMax(RobotIds.CONVEYER_MOTOR_BOTTOM, MotorType.kBrushless);

  /** Creates a new Conveyer. */
  public Conveyer() {
    rollerMotor.restoreFactoryDefaults();
    conveyerMotorTop.restoreFactoryDefaults();
    conveyerMotorBottom.restoreFactoryDefaults();
    rollerMotor.setOpenLoopRampRate(0.1);
    conveyerMotorTop.setOpenLoopRampRate(0.1);
    conveyerMotorBottom.setOpenLoopRampRate(0.1);

  }

  public void setConveyer(double output) {
    rollerMotor.set(output);
    conveyerMotorTop.set(output);
    conveyerMotorBottom.set(output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
