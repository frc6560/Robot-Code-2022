// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.NetworkTable.NtValueDisplay;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import static frc.robot.Constants.*;
import static frc.robot.Constants.PhysicalConstants.*;
import static frc.robot.Constants.ConversionConstants.*;

import static frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;

import javax.swing.plaf.synth.SynthStyle;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  //TODO: Implement PID, you lazy bastard!

  // private final CANSparkMax[] leftMotors = new CANSparkMax[]{
  //   new CANSparkMax(RobotIds.DRIVETRAIN_L_FRONT_MOTOR, MotorType.kBrushless),
  //   new CANSparkMax(RobotIds.DRIVETRAIN_L_BACK_MOTOR, MotorType.kBrushless)
  // };

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

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  private final RelativeEncoder leftEncoder = leftMotors[0].getEncoder();
  private final RelativeEncoder rightEncoder = rightMotors[0].getEncoder();


  private final SimpleMotorFeedforward simpleFFL;
  private final SimpleMotorFeedforward simpleFFR;

  private final AHRS gyro;
  private final DifferentialDriveOdometry odometer;


  private SlewRateLimiter leftLimiter = new SlewRateLimiter(PhysicalConstants.MAX_ACCELERATION);
  private SlewRateLimiter rightLimiter = new SlewRateLimiter(PhysicalConstants.MAX_ACCELERATION);
  private SlewRateLimiter forwardLimiter = new SlewRateLimiter(PhysicalConstants.MAX_ACCELERATION);
  private SlewRateLimiter turnLimiter = new SlewRateLimiter(PhysicalConstants.MAX_ACCELERATION);


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
      .add("r", this::getGyroAngleDegrees);

    ntDispTab("Drivetrain")
      .add("Left Velocity", this::getLVelocityMeters)
      .add("Right Velocity", this::getRVelocityMeters);
      //Add target velocity?


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(gyro.isCalibrating()){
      System.out.println("Still Calibarting!!");
    }
    
    odometer.update(gyro.getRotation2d(), getLPosition(), getRPosition());
    // System.out.println(getCurrentPose());
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

  public double getGyroAngleDegrees() {
    // could multiply this.gyroAngle by 1.03163686 to account for errors associated with gyroscope (From Jack's 2021 code)
    return getGyroAngle().getDegrees();
  }

  public Rotation2d getGyroAngle() {
    return gyro.getRotation2d();
  }

  public double getLPosition() {
    return this.leftEncoder.getPosition() * PhysicalConstants.ROTATIONSTOMETERS;
  }

  public double getRPosition() {
    return this.rightEncoder.getPosition() * PhysicalConstants.ROTATIONSTOMETERS;
  }

  public double getLRpm() {
    return leftEncoder.getVelocity();
  }
  public double getRRpm() {
    return rightEncoder.getVelocity();
  }

  public double getLVelocityFeet() {
    return getLRpm() / DRIVETRAIN_ROTS_PER_FOOT / SECONDS_PER_MINUTE;
  }

  public double getLVelocityMeters(){
    //System.out.println(getLRpm());
    return getLRpm() * PhysicalConstants.RPMTOMETERSPERSEC;
  }

  public double getRVelocityFeet() {
    return getRRpm() / DRIVETRAIN_ROTS_PER_FOOT / SECONDS_PER_MINUTE;
  }

  public double getRVelocityMeters(){
    return getRRpm() * PhysicalConstants.RPMTOMETERSPERSEC;
  }
  
  public DifferentialDriveWheelSpeeds getVelocity(){
    System.out.println("Current speed:");
    System.out.println(new DifferentialDriveWheelSpeeds(getLVelocityMeters(), getRVelocityMeters()));
    return new DifferentialDriveWheelSpeeds(getLVelocityMeters(), getRVelocityMeters());
  }

  public double getLVolts() {
    return leftMotors[0].getAppliedOutput();
  }

  public double getRVolts() {
    return rightMotors[0].getAppliedOutput();
  }

  public void setLRPM(double rpm){
    setLRPM(rpm, 0);
  }
  public void setRRPM(double rpm){
    setRRPM(rpm, 0);
  }

  public void setLRPM(double rpm, double rpms) {
    System.out.println(getLRpm());
    System.out.println(rpm);
    System.out.println("--------------------------");
    for (CANSparkMax motor : leftMotors) {
      motor.getPIDController().setReference(rpm, ControlType.kVelocity, 0, simpleFFL.calculate(rpm), ArbFFUnits.kVoltage);
    }
  }

  public void setRRPM(double rpm, double rpms) {
    for (CANSparkMax motor: rightMotors) {
      motor.getPIDController().setReference(rpm, ControlType.kVelocity, 0, simpleFFR.calculate(rpm, rpms), ArbFFUnits.kVoltage);
    }
  }

  public void setLVelocity(double velocity) {
    setLVelocity(velocity, 0);
    //setLVelocity(velocity, 0);
  }

  public void setRVelocity(double velocity) {
    setRVelocity(velocity, 0);
    //setRVelocity(velocity, 0);
  }

  public void setLVelocity(double velocity, double acceleration) {
    // System.out.println(velocity / PhysicalConstants.RPMTOMETERSPERSEC);
    // System.out.println(velocity);
    // System.out.println(getLRpm());
    // System.out.println("--------------------------");
    setLRPM(leftLimiter.calculate(velocity / PhysicalConstants.RPMTOMETERSPERSEC), acceleration / (PhysicalConstants.RPMTOMETERSPERSEC / 60.0));
  }

  public void setRVelocity(double velocity, double acceleration) {
    setRRPM(rightLimiter.calculate(velocity / PhysicalConstants.RPMTOMETERSPERSEC), acceleration / (PhysicalConstants.RPMTOMETERSPERSEC / 60.0));
  }

  public void setVelocity(double forward, double turn) {
    differentialDrive.arcadeDrive(forwardLimiter.calculate(forward), turnLimiter.calculate(turn));
    //differentialDrive.arcadeDrive(forward, turn);
  }

  public void setTankVelocity(double left, double right) {
    //meters per second
    setLVelocity(left, 0);
    setRVelocity(right, 0);
    
  // left = left / (DRIVETRAIN_ROTS_PER_FOOT / FEET_PER_METER);

  // right = right / (DRIVETRAIN_ROTS_PER_FOOT / FEET_PER_METER);

  //   double leftVel = simpleFFL.calculate(left);
  //   double rightVel = simpleFFR.calculate(right);
    
  //   // System.out.println("Left: " + left + "  vel: " + leftVel);
  //   // System.out.println("Right: " + right + " vel: " + rightVel);
  //   // System.out.println("Actual Left RPM: " + getLRpm());
  //   // System.out.println("Actual Right RPM: " + getRRpm());
    
  //   leftTarget.update(left);
  //   rightTarget.update(right);

  //   for (CANSparkMax motor : leftMotors) {
  //     motor.set(leftVel);
  //   }

    // for (CANSparkMax motor : rightMotors) {
    //   motor.set(rightVel);
    // }
  }

  public DifferentialDrive getDifferentialDrive() {
    return differentialDrive;
  }
  public Pose2d getCurrentPose() {
    //System.out.println(odometer.getPoseMeters());
    return odometer.getPoseMeters();
  }


  // private void setupAllMotorPIDs(PIDController pid) {
  //   for (CANSparkMax motor : leftMotors) {
  //     setupMotorPID(motor, pid);
  //   }
  //   for (CANSparkMax motor : rightMotors) {
  //     setupMotorPID(motor, pid);
  //   }
  // }

  // private void setupMotorPID(CANSparkMax motor, PIDController pid) {
  //   SparkMaxPIDController pidController = motor.getPIDController();

  //   pidController.setP(pid.getP());
  //   pidController.setI(pid.getI());
  //   pidController.setD(pid.getD());

  //   // pidController.setFF(0);
  //   // pidController.setIZone(0);
  //   // pidController.setOutputRange(-1.0, 1.0);
  //   // pidController.setIMaxAccum(0, 0);
  // }

}
