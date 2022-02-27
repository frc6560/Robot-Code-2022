// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

        public static final int DRIVER_STATION_BUTTON_1 = 4;
        public static final int DRIVER_STATION_BUTTON_2 = 1;
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


        public static final int INTAKE_MOTOR = 2;
        public static final int INTAKE_OVERHEAD_MOTOR = 3;
        public static final int INTAKE_SOLENOID = 0;

        public static final int CLIMB_PISTON = 1;
        public static final int CLIMB_ROTATOR_MOTOR = 17;
        public static final int CLIMB_LEFT_EXTENSION_MOTOR = 5;
        public static final int CLIMB_RIGHT_EXTENSION_MOTOR = 14;
    }

    public static final class PhysicalConstants {
        public static final RamseteController RAMSETE_CONTROLLER = new RamseteController(PhysicalConstants.kRamseteB, PhysicalConstants.kRamseteZeta);
        public static final DifferentialDriveKinematics DIFFERENTIAL_DRIVE_KINEMATICS = new DifferentialDriveKinematics(PhysicalConstants.trackWidthMeters);
        public static final double RPMTOMETERSPERSEC = (((0.5 * Math.PI) * 0.305) / 60.0) / 10.38; 
        public static final double ROTATIONSTOMETERS = ((0.5 * Math.PI) * 0.305) / 10.38;
        public static final double RPM_PER_FALCON_UNIT = 10.0 * 60.0 / 2048.0;
        public static final double MAX_HOOD_ENCODER_DISTANCE = 3630.0;
        // TODO: change this constant;
        public static final double DRIVETRAIN_ROTS_PER_FOOT = 1.0 / ((6.0 / 12.0) * Math.PI);

        //TODO: change these constants
        public static final double MAX_SPEED = 1;
        public static final double MAX_TURN_SPEED = 1;

        public static final double MAX_ACCELERATION = 3;

        public static final double trackWidthMeters = 0.70104;

        //Autonomous
        public static final double kRamseteB = 2; //2
        public static final double kRamseteZeta = 0.7; //0.7
    }

    public static final class ConversionConstants {
        public static final double SECONDS_PER_MINUTE = 60.0;
        public static final double FEET_PER_METER = 3.28084;
        public static final Double METERS_TO_FEET = 1.0/FEET_PER_METER;
    }

    public static final class ShooterCalibrations {
        
    public static ShootCalibrationMap SHOOT_CALIBRATION_MAP = new ShootCalibrationMap();
        
        static {
            SHOOT_CALIBRATION_MAP.add(0.0, new ShootCalibrationMap.Trajectory(3000.0, -0.5));
            SHOOT_CALIBRATION_MAP.add(100.0, new ShootCalibrationMap.Trajectory(4700.0, 1));
        }
    }
}
