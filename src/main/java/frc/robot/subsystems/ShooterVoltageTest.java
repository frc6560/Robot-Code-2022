// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.*;

public class ShooterVoltageTest extends SubsystemBase {
  /** Creates a new ShooterVoltageTest. */

  private final CANSparkMax topLeftShooterMotor = new CANSparkMax(5, MotorType.kBrushless);
  //private final CANSparkMax topRightShooterMotor = new CANSparkMax(RobotIds.SHOOTER_MOTOR_RIGHT, MotorType.kBrushless);

  private final CANSparkMax bottomLeftShooterMotor = new CANSparkMax(4, MotorType.kBrushless);
  //private final CANSparkMax bottomRightShooterMotor = new CANSparkMax(RobotIds.SHOOTER_MOTOR_RIGHT, MotorType.kBrushless);



  //private NetworkTableEntry topLeftMotorTableEntry;
  //private NetworkTableEntry topRightMotorTableEntry;
  private NetworkTableEntry topLeftMotorRPM;
  //private NetworkTableEntry topRightMotorRPM;
  private NetworkTableEntry bottomLeftMotorRPM;
  //private NetworkTableEntry bottomRightMotorRPM;
  private NetworkTableEntry bottomMotorTableEntry;
  private NetworkTableEntry topMotorTableEntry;

  public ShooterVoltageTest() {
    topLeftShooterMotor.restoreFactoryDefaults();
    //topRightShooterMotor.restoreFactoryDefaults();
    
    topLeftShooterMotor.setOpenLoopRampRate(0.1);
    //topRightShooterMotor.setOpenLoopRampRate(0.1);

    //topRightShooterMotor.setInverted(false);
    topLeftShooterMotor.setInverted(false);


    bottomLeftShooterMotor.restoreFactoryDefaults();
    //bottomRightShooterMotor.restoreFactoryDefaults();
    
    bottomLeftShooterMotor.setOpenLoopRampRate(0.1);
    //bottomRightShooterMotor.setOpenLoopRampRate(0.1);

    //bottomRightShooterMotor.setInverted(false);
    bottomLeftShooterMotor.setInverted(true);

    // TODO: Calculate optimal PID constants
    // leftShooterMotor.config_kF(0, 0.047197957);
    // leftShooterMotor.config_kP(0, 0.15);
    // leftShooterMotor.config_kI(0, 0.0);
    // leftShooterMotor.config_kD(0, 0.0);

    // rightShooterMotor.config_kF(0, 0.047197957);
    // rightShooterMotor.config_kP(0, 0.15);
    // rightShooterMotor.config_kI(0, 0.0);
    // rightShooterMotor.config_kD(0, 0.0);


    final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Shooter");
    //leftMotorTableEntry = ntTable.getEntry("Left Motor % Output");
    //rightMotorTableEntry = ntTable.getEntry("Right Motor % Output");
    //leftMotorTableEntry.setDouble(0.0);
    //rightMotorTableEntry.setDouble(0.0);

    topMotorTableEntry = ntTable.getEntry("TOP Motor Target RPM");
    topMotorTableEntry.setDouble(0.0);

    bottomMotorTableEntry = ntTable.getEntry("BOTTOM Motor Target RPM");
    bottomMotorTableEntry.setDouble(0.0);
    

    topLeftMotorRPM = ntTable.getEntry("TOP Motor RPM");
    //topRightMotorRPM = ntTable.getEntry("TOP Right Motor RPM");

    bottomLeftMotorRPM = ntTable.getEntry("BOTTOM Motor RPM");
    //bottomRightMotorRPM = ntTable.getEntry("BOTTOM Right Motor RPM");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //double leftOutput = leftMotorTableEntry.getDouble(0) / 100.0;
    //double rightOutput = rightMotorTableEntry.getDouble(0) / 100.0;

    double topOutput = topMotorTableEntry.getDouble(0.0) / 5500.0;
    double bottomOutput = bottomMotorTableEntry.getDouble(0.0) / 5500.0;


    
    topLeftShooterMotor.set(topOutput);
    //topRightShooterMotor.set(topOutput);

    bottomLeftShooterMotor.set(bottomOutput);
    //bottomRightShooterMotor.set(bottomOutput);

    topLeftMotorRPM.setDouble(topLeftShooterMotor.getEncoder().getVelocity());
    // topRightMotorRPM.setDouble(topRightShooterMotor.getEncoder().getVelocity());

    bottomLeftMotorRPM.setDouble(bottomLeftShooterMotor.getEncoder().getVelocity());
    // bottomRightMotorRPM.setDouble(bottomRightShooterMotor.getEncoder().getVelocity());
  
  }
}
