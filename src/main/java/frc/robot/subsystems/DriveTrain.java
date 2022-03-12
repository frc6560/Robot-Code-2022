// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.RobotIds;
import frc.robot.utility.NetworkTable.NtValueDisplay;


public class DriveTrain extends SubsystemBase {
  
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
  

  //check
  private final AHRS gyro = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte) 100);

  private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(PhysicalConstants.MAX_ACCELERATION);
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(PhysicalConstants.MAX_ACCELERATION);

  private final SimpleMotorFeedforward simpleFF = new SimpleMotorFeedforward(PhysicalConstants.KS, PhysicalConstants.KV, PhysicalConstants.KA);

  private double kP, kI, kD, kFF;

  private NetworkTable ntTable;
  private NetworkTableEntry ntPosition, ntspeed, ntP, ntI, ntD, ntFF, ntifTestingVelocity, ntifTestingRotation;


  /** Creates a new DriveTrainLeoGood. */
  public DriveTrain() {
    m_drive.setSafetyEnabled(false);

    setupAllMotors();
    setupEncoders();
    // gyro.calibrate();

    m_odometry = new DifferentialDriveOdometry(new Rotation2d());
    NtValueDisplay.ntDispTab("Drivetrain")
    .add("Degrees", ()->(getAngleContinuous()));


    kP = 0; 
    kI = 0;
    kD = 0; 
    kFF = 0;

    this.ntTable = NetworkTableInstance.getDefault().getTable("Drivetrain");

    ntP = ntTable.getEntry("P");
    ntP.setDouble(0.0);
    ntI = ntTable.getEntry("I");
    ntI.setDouble(0.0);
    ntD = ntTable.getEntry("D");
    ntD.setDouble(0.0);
    ntFF = ntTable.getEntry("FF");
    ntFF.setDouble(0.0);
    ntspeed = ntTable.getEntry("Target Speed");
    ntspeed.setDouble(0.0);

    ntPosition = ntTable.getEntry("Target Position");
    ntPosition.setDouble(0.0);
    
    ntifTestingVelocity = ntTable.getEntry("If testing Postion");
    ntifTestingVelocity.setBoolean(false);

    ntifTestingRotation = ntTable.getEntry("If testing Rotation");
    ntifTestingRotation.setBoolean(false);

    NtValueDisplay.ntDispTab("Drivetrain").add("L Actual Speed", this::getLVelocity).add("R Actual Speed", this::getRVelocity);
    NtValueDisplay.ntDispTab("Drivetrain")
    .add("Actual FF", () -> {return leftMotors[0].getPIDController().getFF();})
    .add("Actual P", () -> {return leftMotors[0].getPIDController().getP();})
    .add("Actual I", () -> {return leftMotors[0].getPIDController().getI();})
    .add("Actual D", () -> {return leftMotors[0].getPIDController().getD();});



  }

  @Override
  public void periodic() {

    System.out.println(gyro.isCalibrating());
    if(!gyro.isConnected())
      DriverStation.reportError("gryo is off", false);
    // System.out.println("hi");
    // This method will be called once per scheduler run
    m_odometry.update(getGyroAngle(), leftEncoder.getPosition(), rightEncoder.getPosition());


    double speed = ntspeed.getDouble(0.0);

    if(ntifTestingVelocity.getBoolean(false)){
      setWheelVelocity(speed, speed);

      updatePID();
    }

    if(ntifTestingRotation.getBoolean(false)){
      double targetPostion = ntPosition.getDouble(0.0);
      setWheelPosition(new double[]{targetPostion, 0.2}, new double[]{-targetPostion, 0.2});
    }
    


  }

  private void updatePID(){
    if(kP != ntP.getDouble(0.0) || kI != ntI.getDouble(0.0) || kD != ntD.getDouble(0.0)){
      System.out.println("Changed!!!!");
      kP = ntP.getDouble(0.0);
      kI = ntI.getDouble(0.0);
      kD = ntD.getDouble(0.0);
      kFF = ntFF.getDouble(0.0);
      setupPID(leftMotors, new PIDController(kP, kI, kD), kFF);
      setupPID(rightMotors, new PIDController(kP, kI, kD), kFF);
    }

  }

  private void setupPID(CANSparkMax[] motors, PIDController newPid, double FF){
    for (CANSparkMax motor : motors) {
      SparkMaxPIDController pidController = motor.getPIDController();

      pidController.setD(newPid.getD());
      pidController.setI(newPid.getI());
      pidController.setP(newPid.getP()); // 1E-5
      pidController.setFF(FF);
      pidController.setIZone(0);
      pidController.setOutputRange(-1.0, 1.0);
      pidController.setIMaxAccum(0, 0);
    }
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

      SparkMaxPIDController pidController = motor.getPIDController();
      pidController.setP(PhysicalConstants.KP);
      pidController.setI(PhysicalConstants.KI);
      pidController.setD(PhysicalConstants.KD);
      pidController.setFF(0);
      pidController.setIZone(0);
      pidController.setOutputRange(-1.0, 1.0);
      pidController.setIMaxAccum(0, 0);
    }

    motors[0].setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);


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
    return new Rotation2d(Math.toRadians(gyro.getRotation2d().getDegrees() * 1.039956786329005));
    // return gyro.getRotation2d();
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getXdistance(){
    return m_odometry.getPoseMeters().getX();
  }

  public double getYdistance(){
    return m_odometry.getPoseMeters().getY();
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
    if(!ntifTestingVelocity.getBoolean(false) && !ntifTestingRotation.getBoolean(false)){
      m_drive.arcadeDrive(forwardLimiter.calculate(forward), turnLimiter.calculate(turn));    //differentialDrive.arcadeDrive(forward, turn);
    }
  }

  public void setWheelVelocity(double left, double right){
    setWheelVelocity(new double[]{left, 0.0}, new double[]{right, 0.0});
  }

  public void setWheelVelocity(double[] left, double[] right){
    setLVelocityMeters(left[0], left[1]);
    setRVelocityMeters(right[0], right[1]);
    // setLVelocityMeters(left[0], 0);
    // setRVelocityMeters(right[0], 0);
    System.out.println("Actual Left Speed: " + getLVelocity());
    System.out.println("Actual right Speed: " + getRVelocity());


    m_drive.feed();

  }

  public void setWheelPosition(double[] left, double[] right){
    setLPositionMeters(left[0], left[1]);
    setRPositionMeters(right[0], right[1]);

    m_drive.feed();

  }


  public void setLVelocityMeters(double velocity, double accel){
    for (CANSparkMax motor : leftMotors) {
      motor.getPIDController().setReference(velocity, ControlType.kVelocity, 0, simpleFF.calculate(velocity, accel), ArbFFUnits.kVoltage);
    }
  }
  public void setRVelocityMeters(double velocity, double accel){
    for (CANSparkMax motor : rightMotors) {
      motor.getPIDController().setReference(velocity, ControlType.kVelocity, 0, simpleFF.calculate(velocity, accel), ArbFFUnits.kVoltage);
    }
  }

  public void setLPositionMeters(double position, double accel){
    for (CANSparkMax motor : leftMotors) {
      motor.getPIDController().setReference(position, ControlType.kPosition, 0, simpleFF.calculate(position, accel), ArbFFUnits.kVoltage);
    }
  }

  public void setRPositionMeters(double position, double accel){
    for (CANSparkMax motor : rightMotors) {
      motor.getPIDController().setReference(position, ControlType.kPosition, 0, simpleFF.calculate(position, accel), ArbFFUnits.kVoltage);
    }
  }

  public double getAngleContinuous(){
    return gyro.getRotation2d().getDegrees() * 1.039956786329005;
  }

  public double getLVelocity(){
    return leftEncoder.getVelocity();
  }

  public double getRVelocity(){
    return rightEncoder.getVelocity();
  }
  
}
