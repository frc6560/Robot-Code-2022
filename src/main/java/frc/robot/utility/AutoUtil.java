// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.io.IOException;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.commands.RamsexyCommand;
import frc.robot.subsystems.DriveTrain;
/** Add your docs here. */
public class AutoUtil {
    private Trajectory trajectory;
    private DriveTrain driveTrain;
    DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(PhysicalConstants.KSVOLTS,
                    PhysicalConstants.KVVOLTSECONDSPERMETER,
                    PhysicalConstants.KAVOLTSECONDSQUARDPERMETER),
            PhysicalConstants.DIFFERENTIAL_DRIVE_KINEMATICS,
            10);
    TrajectoryConfig config = new TrajectoryConfig(PhysicalConstants.MAXSPEEDMETERS,
            PhysicalConstants.MAXACCELERATIONMETERS)
                    .setKinematics(PhysicalConstants.DIFFERENTIAL_DRIVE_KINEMATICS)
                    .addConstraint(autoVoltageConstraint);

        
         Trajectory exampleTrajectory;

        NetworkTable table;
        NetworkTableEntry leftReference;
        NetworkTableEntry leftMeasurement;
        NetworkTableEntry rightReference;
        NetworkTableEntry rightMeasurement;

    public AutoUtil(String filePath, DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        // this.command = pathToCommand(filePath);

        try {
            this.trajectory = TrajectoryUtil
                    .fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(filePath));
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + filePath, ex.getStackTrace());
        }
        table = NetworkTableInstance.getDefault().getTable("troubleshooting");
        leftReference = table.getEntry("left_reference");
        leftMeasurement = table.getEntry("left_measurement");
        rightReference = table.getEntry("right_reference");
        rightMeasurement = table.getEntry("right_measurement");

        config.setReversed(false);
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                    new Translation2d(2, 2),
                    new Translation2d(6, -2)
                    ),
            // End 3 meters straight ahead of where we started, facing forwar
            new Pose2d(8, 0, new Rotation2d(0)),
            // Pass config
            config);

    }

//     public LeoRamsete getCommand() {

//         return pathToCommand();
//     }

//     private LeoRamsete pathToCommand() {
//         Transform2d transform = driveTrain.getPose().minus(exampleTrajectory.getInitialPose());

//         Trajectory newTrajectory = exampleTrajectory.transformBy(transform);

//         // System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!");
//         // System.out.println();
//         // System.out.println(transform);

//         // trajectory = trajectory.transformBy(transform);
//         // System.out.println(trajectory);
//         // System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!");

//         // return new LeoRamsete(
//         // trajectory.transformBy(transform),
//         // driveTrain::getCurrentPose,
//         // // new RamseteController(0, 0),
//         // new RamseteController(PhysicalConstants.kRamseteB,
//         // PhysicalConstants.kRamseteZeta),
//         // new DifferentialDriveKinematics(PhysicalConstants.trackWidthMeters),
//         // (x, y) -> {
//         // //System.out.println("uwu");
//         // driveTrain.setTankVelocity(x, y);
//         // },
//         // driveTrain);

//         // return null;
        

//         PIDController leftController = new PIDController(PhysicalConstants.KP, 0, 0);
//         PIDController rightController = new PIDController(PhysicalConstants.KP, 0, 0);


//         return new LeoRamsete(
//                 newTrajectory,
//                 driveTrain::getPose,
//                 new RamseteController(
//                         PhysicalConstants.kRamseteB,
//                         PhysicalConstants.kRamseteZeta),
//                 new SimpleMotorFeedforward(
//                         PhysicalConstants.KSVOLTS,
//                         PhysicalConstants.KVVOLTSECONDSPERMETER,
//                         PhysicalConstants.KAVOLTSECONDSQUARDPERMETER),
//                 PhysicalConstants.DIFFERENTIAL_DRIVE_KINEMATICS,
//                 driveTrain::getWheelSpeeds,
//                 // new PIDController(12.756, 0, 0),
//                 leftController,
//                 rightController,
//                 (x, y) -> {
//                     System.out.println(x + " " + y);
//                     driveTrain.tankDriveVolts(x, y);

//                     leftMeasurement.setDouble(driveTrain.getWheelSpeeds().leftMetersPerSecond);
//                     leftReference.setDouble(leftController.getSetpoint());
            
//                     rightMeasurement.setDouble(driveTrain.getWheelSpeeds().rightMetersPerSecond);
//                     rightReference.setDouble(rightController.getSetpoint());
//                 },
//                 driveTrain);

//     }

}
