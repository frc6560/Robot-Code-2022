// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.io.IOException;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.commands.LeoRamsete;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class AutoWrapperPathWeaver {
    private Trajectory trajectory;
    private DriveTrain driveTrain;

    NetworkTable table;
    NetworkTableEntry leftReference;
    NetworkTableEntry leftMeasurement;
    NetworkTableEntry rightReference;
    NetworkTableEntry rightMeasurement;

    public AutoWrapperPathWeaver(String pathName, DriveTrain driveTrain) {
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(pathName));
        } catch (Exception ex) {
            DriverStation.reportError("Unable to open trajectory: " + pathName, ex.getStackTrace());
        }
        this.driveTrain = driveTrain;

        table = NetworkTableInstance.getDefault().getTable("troubleshooting");
        leftReference = table.getEntry("left_reference");
        leftMeasurement = table.getEntry("left_measurement");
        rightReference = table.getEntry("right_reference");
        rightMeasurement = table.getEntry("right_measurement");
    }

    // public LeoRamsete getCommand() {
    //     Transform2d transform = driveTrain.getPose().minus(trajectory.getInitialPose());

    //     Trajectory newTrajectory = trajectory.transformBy(transform);

    //     PIDController leftController = new PIDController(PhysicalConstants.KP, 0, 0);
    //     PIDController rightController = new PIDController(PhysicalConstants.KP, 0, 0);

    //     return new LeoRamsete(
    //             newTrajectory,
    //             driveTrain::getPose,
    //             new RamseteController(
    //                     PhysicalConstants.kRamseteB,
    //                     PhysicalConstants.kRamseteZeta),
    //             new SimpleMotorFeedforward(
    //                     PhysicalConstants.KSVOLTS,
    //                     PhysicalConstants.KVVOLTSECONDSPERMETER,
    //                     PhysicalConstants.KAVOLTSECONDSQUARDPERMETER),
    //             PhysicalConstants.DIFFERENTIAL_DRIVE_KINEMATICS,
    //             driveTrain::getWheelSpeeds,
    //             leftController,
    //             rightController,
    //             (x, y) -> {
    //                 driveTrain.tankDriveVolts(x, y);

    //                 System.out.println(x + " " + y);
    //                 leftMeasurement.setDouble(driveTrain.getWheelSpeeds().leftMetersPerSecond);
    //                 leftReference.setDouble(leftController.getSetpoint());

    //                 rightMeasurement.setDouble(driveTrain.getWheelSpeeds().rightMetersPerSecond);
    //                 rightReference.setDouble(rightController.getSetpoint());
    //             },
    //             driveTrain);
    // }

}
