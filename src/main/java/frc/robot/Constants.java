// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.utility.ShootCalibrationMap;

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
        public static final int CONVEYOR_SENSOR = 2;

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
        // TODO: change this constant;
        public static final double DRIVETRAIN_ROTS_PER_FOOT = 6;

        //TODO: change these constants
        public static final double MAX_SPEED = 1;
        public static final double MAX_TURN_SPEED = 1;

        public static final double MAX_ACCELERATION = 3;

        public static final double RPM_PER_FALCON_UNIT = 10.0 * 60.0 / 2048.0;
        public static final double MAX_HOOD_ENCODER_DISTANCE = 3630;
    }

    public static final class ConversionConstants {
        public static final double SECONDS_PER_MINUTE = 60.0;
    }

    public static final class ShooterCalibrations {
        
    public static final ShootCalibrationMap SHOOT_CALIBRATION_MAP = new ShootCalibrationMap();
        
        static {
            SHOOT_CALIBRATION_MAP.add(4.0, new ShootCalibrationMap.Trajectory(4350.0, 2300.0));
            SHOOT_CALIBRATION_MAP.add(6.0, new ShootCalibrationMap.Trajectory(4750.0, 2700.0));
            SHOOT_CALIBRATION_MAP.add(8.0, new ShootCalibrationMap.Trajectory(5250.0, 3050.0));
            SHOOT_CALIBRATION_MAP.add(10.0, new ShootCalibrationMap.Trajectory(5550.0, 3250.0));
            SHOOT_CALIBRATION_MAP.add(12.0, new ShootCalibrationMap.Trajectory(5650.0, 3300.0));
            SHOOT_CALIBRATION_MAP.add(14.0, new ShootCalibrationMap.Trajectory(5700.0, 3350.0));
            SHOOT_CALIBRATION_MAP.add(16.0, new ShootCalibrationMap.Trajectory(5950.0, 3450.0));
            SHOOT_CALIBRATION_MAP.add(18.0, new ShootCalibrationMap.Trajectory(6150.0, 3500.0));
            SHOOT_CALIBRATION_MAP.add(20.0, new ShootCalibrationMap.Trajectory(6350.0, 3600.0));
            SHOOT_CALIBRATION_MAP.add(34.0, new ShootCalibrationMap.Trajectory(6600.0, 3600.0));
            SHOOT_CALIBRATION_MAP.add(37.0, new ShootCalibrationMap.Trajectory(6800.0, 3600.0));
            SHOOT_CALIBRATION_MAP.add(100.0, new ShootCalibrationMap.Trajectory(6800.0, 3600.0));
        }
    }
}
