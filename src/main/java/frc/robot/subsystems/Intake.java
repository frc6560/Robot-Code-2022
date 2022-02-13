// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import frc.robot.Constants.RobotIds;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax(RobotIds.INTAKE_BOTTOM_MOTOR, MotorType.kBrushless);
  private final Solenoid intakePiston = new Solenoid(PneumaticsModuleType.CTREPCM, RobotIds.INTAKE_SOLENOID);
  private final CANSparkMax hopperMotor = new CANSparkMax(RobotIds.INTAKE_HOPPER_MOTOR, MotorType.kBrushless);

  private double requestedIntakeMotorOutput = 0.0;
  private double requestedHopperMotorOutput = 0.0;
  private int downFrames = 0;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setOpenLoopRampRate(0.1);

    hopperMotor.restoreFactoryDefaults();
    hopperMotor.setOpenLoopRampRate(0.1);

  }

  public void setIntakeMotorOutput(double output) {
    requestedIntakeMotorOutput = output;
  }

  public void setHopperMotorOutput(double output) {
    requestedHopperMotorOutput = output;
  }

  public void setPiston(boolean out) {
    intakePiston.set(out);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (intakePiston.get()) {
      downFrames++;
    } else {
      downFrames = 0;
    }
    
    intakeMotor.set(requestedHopperMotorOutput);
    

    //intakeMotor.set(canRunIntakeMotor() ? requestedIntakeMotorOutput : 0.0);
    //hopperMotor.set(canRunIntakeMotor() ? requestedHopperMotorOutput : 0.0);
  }
  
  private boolean canRunIntakeMotor() {
    if (downFrames >= 35) {
      return true;
    } else {
      return false;
    }
  }
}
