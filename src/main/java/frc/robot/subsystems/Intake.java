// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import frc.robot.Constants.RobotIds;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax(RobotIds.INTAKE_MOTOR, MotorType.kBrushless);
  private final Solenoid intakePiston = new Solenoid(PneumaticsModuleType.CTREPCM, RobotIds.INTAKE_SOLENOID);

  private final int minFramesDown = 20;

  private double targetIntakeMotorOutput = 0.7;

  private boolean reversed = false;

  private final NetworkTable ntTable;

  private int downFrames = 0;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setOpenLoopRampRate(0.1);
    intakeMotor.setInverted(false);
    
    ntTable = NetworkTableInstance.getDefault().getTable("Intake");
  }

  public void setIntakeMotorOutput(double output) {
    targetIntakeMotorOutput = output * (reversed ? -1: 1);
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
    
    intakeMotor.set(canRunIntakeMotor() ? targetIntakeMotorOutput : 0.0);
  }
  
  private boolean canRunIntakeMotor() {
    if (downFrames >= minFramesDown) {
      return true;
    } else {
      return false;
    }
  }
  
  public void setReversed(boolean state){
    this.reversed = state;
  }
}
