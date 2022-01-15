// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;


import static frc.robot.Constants.*;
import frc.robot.utility.HeadingConversion;
import static frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  //TODO: Implement PID, you lazy bastard!

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


  private final SimpleMotorFeedforward simpleFFL;
  private final SimpleMotorFeedforward simpleFFR;

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

    simpleFFL = new SimpleMotorFeedforward(0.12826078, 0.00210809109, 0.0004);
    simpleFFR = new SimpleMotorFeedforward(0.12032416, 0.00209926402, 0.0004);


    ntDispTab("Odometer")
      .add("x", () -> odometer.getPoseMeters().getX())
      .add("y", () -> odometer.getPoseMeters().getY())
      .add("r", () -> odometer.getPoseMeters().getRotation().getDegrees())
      .add("rCont", () -> Math.toRadians(getGyroAngleDegrees()))
      .add("lPos", this::getLPosition)
      .add("rPos", this::getRPosition);

    ntDispTab("Drivetrain")
      .add("Left Velocity", this::getLVelocity)
      .add("Right Velocity", this::getRVelocity)
      .add("Right Volts", this::getLVolts)
      .add("Left Volts", this::getRVolts);
      //Add target velocity?


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.gyroAngle = gyro.getYaw();
    this.totalGyroAngle = headingConverter.getTotalHeading(this.gyroAngle);

    odometer.update(getGyroAngle(), getLPosition(), getRPosition());

    
  }

  private void setupAllMotors() {
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
    return Rotation2d.fromDegrees(getGyroAngleDegrees());
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

  public double getLVolts() {
    return leftMotors[0].getAppliedOutput();
  }

  public double getRVolts() {
    return rightMotors[0].getAppliedOutput();
  }

  private void setupAllMotorPIDs(PIDController pid) {
    for (CANSparkMax motor : leftMotors) {
      setupMotorPID(motor, pid);
    }
    for (CANSparkMax motor : rightMotors) {
      setupMotorPID(motor, pid);
    }
  }

  private void setupMotorPID(CANSparkMax motor, PIDController pid) {
    SparkMaxPIDController pidController = motor.getPIDController();

    pidController.setP(pid.getP());
    pidController.setI(pid.getI());
    pidController.setD(pid.getD());

    // pidController.setFF(0);
    // pidController.setIZone(0);
    // pidController.setOutputRange(-1.0, 1.0);
    // pidController.setIMaxAccum(0, 0);
  }

}
