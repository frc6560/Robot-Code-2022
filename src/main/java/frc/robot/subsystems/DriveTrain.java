// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  private final double kMaxSpeed = 3.0; // m/s TODO: update this

  // TODO: Update values
  private final Translation2d m_frontLeftLocation = new Translation2d(0.69, 0.69);
  private final Translation2d m_frontRightLocation = new Translation2d(0.69, -0.69);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.69, 0.69);
  private final Translation2d m_backRightLocation = new Translation2d(-0.69, -0.69);

  private SwerveModule m_frontLeft;
  private SwerveModule m_frontRight;
  private SwerveModule m_backLeft;
  private SwerveModule m_backRight;

  private AHRS gyro;


  private SwerveDriveKinematics m_kinematics;
  private SwerveDriveOdometry m_odometry;

  private SwerveDrivePoseEstimator poseEstimator;

  private final NetworkTableEntry ntL = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tL");
  private final NetworkTableEntry ntX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
  private final NetworkTableEntry ntV = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
  private final NetworkTableEntry ntTurret = NetworkTableInstance.getDefault().getTable("Shooter").getEntry("Actual Turret");

  private volatile SwerveModuleState[] currentSwerveModuleStates;


  public DriveTrain() {
    
    m_frontLeft = new SwerveModule(new TalonFX(8), new CANSparkMax(5, MotorType.kBrushless));
    m_frontRight = new SwerveModule(new TalonFX(10), new CANSparkMax(13, MotorType.kBrushless));
    m_backLeft = new SwerveModule(new TalonFX(22), new CANSparkMax(14, MotorType.kBrushless));
    m_backRight = new SwerveModule(new TalonFX(27), new CANSparkMax(20, MotorType.kBrushless));

    gyro = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte) 100);
    
    gyro.calibrate();
    gyro.reset();

    m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    m_odometry = new SwerveDriveOdometry(m_kinematics, gyro.getRotation2d());

    // TODO: robot specific constants: update values
    poseEstimator = new SwerveDrivePoseEstimator(
      gyro.getRotation2d(),
      new Pose2d(),
      m_kinematics,
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(Units.degreesToRadians(0.01)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));      
  }

  public SwerveDriveKinematics getKinematics() {
    return this.m_kinematics;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(currentSwerveModuleStates);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rotationIsPosition) {
    currentSwerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(currentSwerveModuleStates, kMaxSpeed);

    if (rotationIsPosition) {
      for (SwerveModuleState i : currentSwerveModuleStates) {
        i.angle = new Rotation2d(rot);
      }
    }

    setModuleStates(currentSwerveModuleStates);
  }

  public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative, boolean rotationIsPosition) {
    drive(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, fieldRelative, rotationIsPosition);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_backLeft.setDesiredState(states[2]);
    m_backRight.setDesiredState(states[3]);
  }

  public void stop() {
    drive(0.0, 0.0, 0.0, false, false);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
      gyro.getRotation2d(),
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_backLeft.getState(),
      m_backRight.getState()
    );

    resetOdometryWithVision();

    poseEstimator.addVisionMeasurement(
      m_odometry.getPoseMeters(),
      Timer.getFPGATimestamp() - getLimelightLatency()
    );
  }

  public void resetOdometryWithVision() {
    if (ntV.getDouble(0.0) > 0) {
      Rotation2d initialRot = m_odometry.getPoseMeters().getRotation();
      Rotation2d limelightRot = Rotation2d.fromDegrees(ntX.getDouble(0.0));
      Rotation2d hoodRot = Rotation2d.fromDegrees(ntTurret.getDouble(0.0));

      Rotation2d newRotation = initialRot.plus(limelightRot).plus(hoodRot);

      m_odometry.resetPosition(new Pose2d(m_odometry.getPoseMeters().getTranslation(), newRotation), gyro.getRotation2d());
    }
  }

  private double getLimelightLatency() {
    return ntL.getDouble(0.0) + 11; // >11 ms is constant recommended by limelight accounting for extra latency
  }

  public double getAngleContinuous() {
    return gyro.getRotation2d().getDegrees() * 1.039956786329005;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose, Rotation2d gyroAngle) {
    m_odometry.resetPosition(pose, gyroAngle);
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, this.gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
