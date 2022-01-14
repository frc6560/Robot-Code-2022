/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utility;

import java.util.ArrayList;
import java.util.List;

/**
 * Add your docs here.
 */
public class ShootCalibrationMap {
    public static class OutOfBoundsException extends Throwable {
        private static final long serialVersionUID = 5786658266510778451L;

    }

    public static class Trajectory {
        public final double shooterRpm;
        public final double hoodPos;
    
        public Trajectory(double shooterRpm, double hoodPos){
            this.shooterRpm = shooterRpm;
            this.hoodPos = hoodPos;
        }
    }

    private static class Point {
        public final double distance;
        public final Trajectory trajectory;

        public Point(double distance, Trajectory trajectory) {
            this.distance = distance;
            this.trajectory = trajectory;
        }
    }

    private final List<Point> points = new ArrayList<>();

    public Trajectory get(double distance) throws OutOfBoundsException {
        int highIndex = -1;
        
        for(int i = 0; i < points.size(); i++){
            if(distance <= points.get(i).distance){
                highIndex = i;
                break;
            }
        }

        int lowIndex = highIndex - 1;

        if (lowIndex < 0 || highIndex >= points.size()) {
            throw new OutOfBoundsException();
        }

        Point lowPoint = points.get(lowIndex);
        Point highPoint = points.get(highIndex);

        double stepDiff = highPoint.distance - lowPoint.distance;
        double weightFactor = (distance - lowPoint.distance) / stepDiff;

        double minHoodPos = lowPoint.trajectory.hoodPos;
        double maxHoodPos = highPoint.trajectory.hoodPos;

        double minShooterRpm = lowPoint.trajectory.shooterRpm;
        double maxShooterRpm = highPoint.trajectory.shooterRpm;

        double shooterRpm = minShooterRpm * (1.0-weightFactor) + maxShooterRpm * (weightFactor);
        double hoodPos = minHoodPos * (1.0-weightFactor) + maxHoodPos * (weightFactor);

        return new Trajectory(shooterRpm, hoodPos);
    }

    public void add(double distance, Trajectory trajectory) {
        points.add(new Point(distance, trajectory));

        points.sort((Point p1, Point p2) -> {
            if (p1.distance < p2.distance) return -1;
            if (p1.distance > p2.distance) return 1;
            return 0;
        });
    }
}
