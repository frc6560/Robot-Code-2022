// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.RobotIds;
import frc.robot.utility.NetworkTable.NtValueDisplay;


public class DriveTrainLeoGood extends SubsystemBase {
  
  private final CANSparkMax[] leftMotors = new CANSparkMax[] {
    new CANSparkMax(RobotIds.DRIVETRAIN_L_FRONT_MOTOR, MotorType.kBrushless),
    new CANSparkMax(RobotIds.DRIVETRAIN_L_BACK_MOTOR, MotorType.kBrushless)
  };

  private final CANSparkMax[] rightMotors = new CANSparkMax[]{
    new CANSparkMax(RobotIds.DRIVETRAIN_R_FRONT_MOTOR, MotorType.kBrushless),
    new CANSparkMax(RobotIds.DRIVETRAIN_R_BACK_MOTOR, MotorType.kBrushless)
  };

  private final MotorControllerGroup leftMotorGroup = new MotorControllerGroup(leftMotors);
  private final MotorControllerGroup rightMotorGroup = new MotorControllerGroup(rightMotors);


  private final DifferentialDrive m_drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  private final RelativeEncoder leftEncoder = leftMotors[0].getEncoder();
  private final RelativeEncoder rightEncoder = rightMotors[0].getEncoder();

  private final DifferentialDriveOdometry m_odometry;
  
  private NetworkTable nTable;

  //check
  private final AHRS gyro = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte) 100);


  /** Creates a new DriveTrainLeoGood. */
  public DriveTrainLeoGood() {
    nTable = NetworkTableInstance.getDefault().getTable("troubleshooting");

    setupAllMotors();
    setupEncoders();
    gyro.calibrate();

    m_odometry = new DifferentialDriveOdometry(new Rotation2d());
    NtValueDisplay.ntDispTab("Drivetrain")
    .add("bob", ()->(m_odometry.getPoseMeters().getRotation().getDegrees()));

  }

  public void resetGhyro(){
    gyro.reset();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(getGyroAngle(), leftEncoder.getPosition(), rightEncoder.getPosition());

    NetworkTableEntry m_xEntry = nTable.getEntry("Left");
    NetworkTableEntry m_yEntry = nTable.getEntry("RIght");



    m_xEntry.setNumber(leftEncoder.getPosition());
    m_yEntry.setNumber(rightEncoder.getPosition());
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
      motor.setIdleMode(IdleMode.kBrake);
    }

    // Changes default motor controller "send speed" from 20ms to 10ms
    // Add if having issues with accuracy
    //motors[0].setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);

  }

  private void setupEncoders(){

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    leftEncoder.setPositionConversionFactor(PhysicalConstants.ROTATIONSTOMETERS);
    rightEncoder.setPositionConversionFactor(PhysicalConstants.ROTATIONSTOMETERS);

    leftEncoder.setVelocityConversionFactor(PhysicalConstants.RPMTOMETERSPERSEC);
    rightEncoder.setVelocityConversionFactor(PhysicalConstants.RPMTOMETERSPERSEC);
  }

  private Rotation2d getGyroAngle(){
    return new Rotation2d(Math.toRadians(gyro.getRotation2d().getDegrees() * 1.040763226366002));
    // return gyro.getRotation2d();
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {

    leftMotorGroup.setVoltage(leftVolts);

    rightMotorGroup.setVoltage(rightVolts);

    m_drive.feed();

  }

  public DifferentialDrive getDifferentialDrive(){
    return m_drive;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }

  public void setVelocity(double forward, double turn) {
    m_drive.arcadeDrive(forward, turn);
    //differentialDrive.arcadeDrive(forward, turn);
  }
  
}
