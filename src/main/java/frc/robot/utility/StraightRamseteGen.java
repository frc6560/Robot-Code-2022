// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.commands.RamsexyCommand;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class StraightRamseteGen implements AutoWrapperInterface {
        private final DriveTrain driveTrain;
        private Trajectory trajectory;

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

        public StraightRamseteGen(DriveTrain driveTrain, double targetLength) {
                this.driveTrain = driveTrain;
                config.setReversed(true);
                trajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(),
                                // End 3 meters straight ahead of where we started, facing forwar
                                new Pose2d(targetLength * Math.copySign(1, targetLength), 0, new Rotation2d(0)),
                                // Pass config
                                config);
        }

        public RamsexyCommand getCommand() {

                PIDController leftController = new PIDController(PhysicalConstants.KP, 0, 0);
                PIDController rightController = new PIDController(PhysicalConstants.KP, 0, 0);

                return new RamsexyCommand(
                                driveTrain::getPose,
                                new RamseteController(
                                                PhysicalConstants.kRamseteB,
                                                PhysicalConstants.kRamseteZeta),
                                new SimpleMotorFeedforward(
                                                PhysicalConstants.KSVOLTS,
                                                PhysicalConstants.KVVOLTSECONDSPERMETER,
                                                PhysicalConstants.KAVOLTSECONDSQUARDPERMETER),
                                PhysicalConstants.DIFFERENTIAL_DRIVE_KINEMATICS,
                                driveTrain::getWheelSpeeds,
                                leftController,
                                rightController,
                                (x, y) -> {
                                        driveTrain.tankDriveVolts(x, y);
                                },
                                this,
                                driveTrain);

        }

        @Override
        public Trajectory getTrajectory() {
                Transform2d transform = driveTrain.getPose().minus(trajectory.getInitialPose());

                Trajectory newTrajectory = trajectory.transformBy(transform);
                return newTrajectory;
        }
}
