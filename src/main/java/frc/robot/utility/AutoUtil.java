// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.io.IOException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.commands.LeoRamsete;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class AutoUtil {
    private Trajectory trajectory;
    private DriveTrain driveTrain;
    private LeoRamsete command;
    private String filePath;

    public AutoUtil(String filePath, DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.command = pathToCommand(filePath);  
        this.filePath = filePath;
    }

    public Trajectory getTrajectory(){
        return trajectory;
    }

    public LeoRamsete getCommand(){
        return pathToCommand(filePath);
    }


    private LeoRamsete pathToCommand(String filePath) {
        try {
            if(trajectory == null)
                trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(filePath));
            Transform2d transform = driveTrain.getCurrentPose().minus(trajectory.getInitialPose());   
            System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!"); 
            System.out.println();
            System.out.println(transform);   
  
            // trajectory = trajectory.transformBy(transform);
            // System.out.println(trajectory);
            // System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!"); 
            
            return new LeoRamsete(
                trajectory.transformBy(transform),
                driveTrain::getCurrentPose,
                // new RamseteController(0, 0),
                new RamseteController(PhysicalConstants.kRamseteB, PhysicalConstants.kRamseteZeta),
                new DifferentialDriveKinematics(PhysicalConstants.trackWidthMeters),                
                (x, y) -> {
                    //System.out.println("uwu");
                    driveTrain.setTankVelocity(x, y);
                },
                driveTrain);

            // return new LeoRamsete(
            //     trajectory.transformBy(transform), 
            //     driveTrain::getCurrentPose, 
            //     new RamseteController(PhysicalConstants.kRamseteB, PhysicalConstants.kRamseteZeta),
            //     // new SimpleMotorFeedforward(0, 0, 0),
            //     new SimpleMotorFeedforward(0.08691, 2, 0.043694), //(0.18691, 2.7613, 0.43694)
            //     new DifferentialDriveKinematics(PhysicalConstants.trackWidthMeters),
            //     driveTrain::getVelocity, 
            //     //new PIDController(12.756, 0, 0),
            //     new PIDController(3, 0, 0),
            //     new PIDController(3, 0, 0), 
            //     driveTrain::setTankVolts, 
            //     driveTrain);


        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + filePath, ex.getStackTrace());
            return null;
        }
    }

}
