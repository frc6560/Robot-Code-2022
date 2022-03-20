// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.utility.AutoWrapper;
import frc.robot.utility.ShootCalibrationMap;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public static final class ControllerIds {
        public static final int FIRST_DRIVER_CONTROLLER = 0;
        public static final int SECOND_DRIVER_CONTROL_STATION = 1;
        public static final int SECOND_DRIVER_CONTROLLER = 2;

        public static final int XBOX_L_JOY_X = 0;
        public static final int XBOX_L_JOY_Y = 1;

        public static final int XBOX_R_JOY_X = 4;
        public static final int XBOX_R_JOY_Y = 5;

        public static final int XBOX_L_BUMPER = 5;
        public static final int XBOX_R_BUMPER = 6;

        public static final int XBOX_L_TRIGGER = 2;
        public static final int XBOX_R_TRIGGER = 3;

        public static final int XBOX_Y_BUTTON = 4;
        public static final int XBOX_X_BUTTON = 3;
        public static final int XBOX_B_BUTTON = 2;
        public static final int XBOX_A_BUTTON = 1;

        public static final int DRIVER_STATION_TOGGLE_1 = 2;
        public static final int DRIVER_STATION_TOGGLE_2 = 5;
        public static final int DRIVER_STATION_TOGGLE_3 = 6;
        public static final int DRIVER_STATION_TOGGLE_4 = 9;

        public static final int DRIVER_STATION_BUTTON_1 = 1;
        public static final int DRIVER_STATION_BUTTON_2 = 4;
        public static final int DRIVER_STATION_BUTTON_3 = 3;

        public static final int DRIVER_STATION_X_AXIS = 0;
        public static final int DRIVER_STATION_Y_AXIS = 1;
    }


    public static final class RobotIds {
        
        //TODO: Negative Motor controller ids are tbd
        public static final int DRIVETRAIN_R_FRONT_MOTOR = 1;
        public static final int DRIVETRAIN_R_BACK_MOTOR = 20;

        public static final int DRIVETRAIN_L_FRONT_MOTOR = 19;
        public static final int DRIVETRAIN_L_BACK_MOTOR = 18;

        public static final int CONVEYOR_MOTOR_TOP = 16;
        public static final int CONVEYOR_MOTOR_BOTTOM = 15;
        public static final int CONVEYOR_SENSOR = 0;

        public static final int SHOOTER_MOTOR_LEFT = 21;
        public static final int SHOOTER_MOTOR_RIGHT = 22;

        public static final int SHOOTER_HOOD_ACTUATOR_LEFT = 0;
        public static final int SHOOTER_HOOD_ACTUATOR_RIGHT = 1;
        public static final int SHOOTER_HOOD_ENCODER_A = 8;
        public static final int SHOOTER_HOOD_ENCODER_B = 11;
        public static final int SHOOTER_TURRET_MOTOR = 4;


        public static final int INTAKE_MOTOR = 17;
        public static final int INTAKE_OVERHEAD_MOTOR = 3;
        public static final int INTAKE_SOLENOID = 0;

        public static final int CLIMB_PISTON = 3;
        public static final int CLIMB_ROTATOR_PISTON = 2; //TODO: set constant
        public static final int CLIMB_LEFT_EXTENSION_MOTOR = 10;
        public static final int CLIMB_RIGHT_EXTENSION_MOTOR = 43;
    }

    public static final class PhysicalConstants {
        // public static final double KSVOLTS = 0.18085;
        // public static final double KVVOLTSECONDSPERMETER = 2.7552;
        // public static final double KAVOLTSECONDSQUARDPERMETER = 0.73896;
        // public static final double KP = 2.4691;

        public static final double KS = 0.18053;
        public static final double KV = 2.1727;
        public static final double KA = 0.23044;
        public static final double KP = 0.000014; //0.000014
        public static final double KI = 0.000000003; //0.000000003
        public static final double KD = 0.0000;



        public static final double MAXSPEEDMETERS = 6; //Three ball: 1 //Four ball: 4
        public static final double MAXACCELERATIONMETERS = 0.9; //Three ball:1 //Four ball: 1


         //Autonomous
        public static final double kRamseteB = 2; //2
        public static final double kRamseteZeta = 0.7; //0.7


        public static final RamseteController RAMSETE_CONTROLLER = new RamseteController(PhysicalConstants.kRamseteB, PhysicalConstants.kRamseteZeta);
        public static final DifferentialDriveKinematics DIFFERENTIAL_DRIVE_KINEMATICS = new DifferentialDriveKinematics(PhysicalConstants.trackWidthMeters);
        // public static final double ROTATIONSTOMETERS = (((0.5 * Math.PI) * 0.3048) / 10.384615384615384615384615384615) * 1.293518518518519;
        public static final double ROTATIONSTOMETERS = (((0.5 * Math.PI) * 0.3048) / 8.0);
        public static final double RPMTOMETERSPERSEC = ROTATIONSTOMETERS / 60.0; 
        public static final double RPM_PER_FALCON_UNIT = 10.0 * 60.0 / 2048.0;
        public static final double MAX_HOOD_ENCODER_DISTANCE = 3630.0;
        // TODO: change this constant;
        public static final double DRIVETRAIN_ROTS_PER_FOOT = 1.0 / ((6.0 / 12.0) * Math.PI);

        public static final double CLIMB_EXTENSION_INCHES_PER_ROTATION = (1.0 / 5) * 0.5; // 1 rotation / 5 rot per bigRot * 0.5 inch per bigRot

        //TODO: change these constants
        public static final double MAX_SPEED = 1;
        public static final double MAX_TURN_SPEED = 1;

        public static final double MAX_ACCELERATION = 13;

        public static final double trackWidthMeters = 0.708025;

       
    }

    public static final class ConversionConstants {
        public static final double SECONDS_PER_MINUTE = 60.0;
        public static final double FEET_TO_METER = 0.3048;
        public static final double METERS_TO_FEET = 1.0/FEET_TO_METER;
    }

    public static final class ShooterCalibrations {
        
    public static ShootCalibrationMap SHOOT_CALIBRATION_MAP = new ShootCalibrationMap();
    public static final double ANGLE_ADJUSTMENT_CONSTANT = -0.05;
    public static final double SPEED_ADJUSTMENT_CONSTANT = 80;
        
        static {

            SHOOT_CALIBRATION_MAP.add(100.0, new ShootCalibrationMap.Trajectory(3000.0 + SPEED_ADJUSTMENT_CONSTANT, -1 + ANGLE_ADJUSTMENT_CONSTANT));      //Degrees   Distance
            
            SHOOT_CALIBRATION_MAP.add(4.968, new ShootCalibrationMap.Trajectory(3200.0 + SPEED_ADJUSTMENT_CONSTANT, -0.35 + ANGLE_ADJUSTMENT_CONSTANT));   // 4.968      9.2
            SHOOT_CALIBRATION_MAP.add(2.5746, new ShootCalibrationMap.Trajectory(3200.0 + SPEED_ADJUSTMENT_CONSTANT, -0.2+ ANGLE_ADJUSTMENT_CONSTANT));   // 2.5746    10.1
            SHOOT_CALIBRATION_MAP.add(0.1811, new ShootCalibrationMap.Trajectory(3280.0 + SPEED_ADJUSTMENT_CONSTANT, -0.15+ ANGLE_ADJUSTMENT_CONSTANT));  // 0.1811     11
            SHOOT_CALIBRATION_MAP.add(-2.478, new ShootCalibrationMap.Trajectory(3320.0 + SPEED_ADJUSTMENT_CONSTANT, -0.1+ ANGLE_ADJUSTMENT_CONSTANT));   // -2.478     12
            SHOOT_CALIBRATION_MAP.add(-5.138, new ShootCalibrationMap.Trajectory(3400.0 + SPEED_ADJUSTMENT_CONSTANT, -0.1+ ANGLE_ADJUSTMENT_CONSTANT));   // -5.138     13
            SHOOT_CALIBRATION_MAP.add(-7.531, new ShootCalibrationMap.Trajectory(3570.0 + SPEED_ADJUSTMENT_CONSTANT, 0.0+ ANGLE_ADJUSTMENT_CONSTANT));    // -7.531    13.9
            SHOOT_CALIBRATION_MAP.add(-10.456, new ShootCalibrationMap.Trajectory(3600.0 + SPEED_ADJUSTMENT_CONSTANT, 0.15+ ANGLE_ADJUSTMENT_CONSTANT));  // -7.531    15.0

            SHOOT_CALIBRATION_MAP.add(-100, new ShootCalibrationMap.Trajectory(4700.0 + SPEED_ADJUSTMENT_CONSTANT, 1 + ANGLE_ADJUSTMENT_CONSTANT));




            // SHOOT_CALIBRATION_MAP.add(100.0, new ShootCalibrationMap.Trajectory(3000.0, -1));      //Degrees   Distance
            
            // SHOOT_CALIBRATION_MAP.add(4.968, new ShootCalibrationMap.Trajectory(3200.0, -0.35));   // 4.968      9.2
            // SHOOT_CALIBRATION_MAP.add(2.5746, new ShootCalibrationMap.Trajectory(3200.0, -0.2));   // 2.5746    10.1
            // SHOOT_CALIBRATION_MAP.add(0.1811, new ShootCalibrationMap.Trajectory(3280.0, -0.15));  // 0.1811     11
            // SHOOT_CALIBRATION_MAP.add(-2.478, new ShootCalibrationMap.Trajectory(3320.0, -0.1));   // -2.478     12
            // SHOOT_CALIBRATION_MAP.add(-5.138, new ShootCalibrationMap.Trajectory(3400.0, -0.1));   // -5.138     13
            // SHOOT_CALIBRATION_MAP.add(-7.531, new ShootCalibrationMap.Trajectory(3570.0, 0.0));    // -7.531    13.9
            // SHOOT_CALIBRATION_MAP.add(-10.456, new ShootCalibrationMap.Trajectory(3600.0, 0.15));  // -7.531    15.0

            // SHOOT_CALIBRATION_MAP.add(-100, new ShootCalibrationMap.Trajectory(4700.0, 1));
        }
    }
    /*
 
            // SHOOT_CALIBRATION_MAP.add(9.2, new ShootCalibrationMap.Trajectory(3200.0, -0.35));
            // SHOOT_CALIBRATION_MAP.add(10.1, new ShootCalibrationMap.Trajectory(3200.0, -0.2));
            // SHOOT_CALIBRATION_MAP.add(11, new ShootCalibrationMap.Trajectory(3280.0, -0.15));
            // SHOOT_CALIBRATION_MAP.add(12, new ShootCalibrationMap.Trajectory(3320.0, -0.1));
            // SHOOT_CALIBRATION_MAP.add(13, new ShootCalibrationMap.Trajectory(3400.0, -0.1));
            // SHOOT_CALIBRATION_MAP.add(13.9, new ShootCalibrationMap.Trajectory(3500.0, 0.0));

    */
    
}
