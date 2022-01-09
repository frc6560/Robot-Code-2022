// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
//import frc.robot.utility.NetworkTable.NtPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;


import static frc.robot.Constants.*;
import frc.utility.HeadingConversion;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */



  private final CANSparkMax[] leftMotors = new CANSparkMax[]{
    new CANSparkMax(RobotIds.DRIVETRAIN_L_FRONT_MOTOR, MotorType.kBrushless),
    new CANSparkMax(RobotIds.DRIVETRAIN_L_BACK_MOTOR, MotorType.kBrushless)
  };

  private final CANSparkMax[] rightMotors = new CANSparkMax[]{
    new CANSparkMax(RobotIds.DRIVETRAIN_R_FRONT_MOTOR, MotorType.kBrushless),
    new CANSparkMax(RobotIds.DRIVETRAIN_R_BACK_MOTOR, MotorType.kBrushless)
  };
  
  private final RelativeEncoder leftEncoder = leftMotors[0].getEncoder();
  private final RelativeEncoder rightEncoder = rightMotors[0].getEncoder();

  private final AHRS gyro;
  private final DifferentialDriveOdometry odometer;
  private final HeadingConversion headingConverter = new HeadingConversion();


  private double gyroAngle = 0.0;
  private double totalGyroAngle = 0.0;


  public DriveTrain() {
    setupAllMotors();

    gyro = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte) 100);
    gyro.calibrate();

    odometer = new DifferentialDriveOdometry(getGyroAngle());

    // TODO: add SimpleFFL & SimpleFFR (SimpleMotorFeedForward)

    // TODO: add odometer ntDisplayTab
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.gyroAngle = gyro.getYaw();
    this.totalGyroAngle = headingConverter.getTotalHeading(this.gyroAngle);

    odometer.update(getGyroAngle(), getLPosition(), getRPosition());
  }

  private void setupAllMotors() {
    // format: setupMotors(motors[], inverted)
    // inversion tbd
    setupMotors(leftMotors, false);
    setupMotors(rightMotors, true);
  }

  private void setupMotors(CANSparkMax[] motors, boolean inverted) {
    for (CANSparkMax motor : motors) {
      motor.restoreFactoryDefaults();
      motor.setInverted(inverted);
      motor.getEncoder().setPosition(0);
      motor.setClosedLoopRampRate(0.0);
    }

    // Changes default motor controller "send speed" from 20ms to 10ms
    // Add if having issues with accuracy
    //motors[0].setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);

  }

  public double getGyroAngleDegrees() {
    // could multiply this.gyroAngle by 1.03163686 to account for errors associated with gyroscope (From Jack's 2021 code)
    return this.totalGyroAngle;
  }

  public Rotation2d getGyroAngle() {
    // could multiply this.gyroAngle by 1.03163686 to account for errors associated with gyroscope (From Jack's 2021 code)
    return Rotation2d.fromDegrees(this.totalGyroAngle);
  }

  public double getLPosition() {
    return this.leftEncoder.getPosition() / PhysicalConstants.DRIVETRAIN_ROTS_PER_FOOT;
  }

  public double getRPosition() {
    return this.rightEncoder.getPosition() / PhysicalConstants.DRIVETRAIN_ROTS_PER_FOOT;
  }
  

  public double getLRpm() {
    return leftEncoder.getVelocity();
  }
  public double getRRpm() {
    return rightEncoder.getVelocity();
  }

  public double getLVelocity() {
    return getLRpm() / PhysicalConstants.DRIVETRAIN_ROTS_PER_FOOT / ConversionConstants.SECONDS_PER_MINUTE;
  }

  public double getRVelocity() {
    return getRRpm() / PhysicalConstants.DRIVETRAIN_ROTS_PER_FOOT / ConversionConstants.SECONDS_PER_MINUTE;
  }

}