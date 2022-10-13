/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utility;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import frc.robot.subsystems.Limelight;

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

        @Override
        public String toString() {
            return "shooterRpm=" + shooterRpm + ",hoodPos=" + hoodPos;
        }
    }

    private static class Point {
        public final double distance;
        public final Trajectory trajectory;

        public Point(double distance, Trajectory trajectory) {
            this.distance = distance;
            this.trajectory = trajectory;
        }

        @Override
        public String toString() {
            return "distance=" + distance + "," + trajectory.toString();
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

    public Trajectory get(double distance, double delta, double zeta) throws OutOfBoundsException {
        Trajectory original = get(distance);
        
        
        // double slope = (highPoint.trajectory.shooterRpm - lowPoint.trajectory.shooterRpm) / (highPoint.distance - lowPoint.distance);
        double adjustment = delta + (delta / 100) * zeta * (distance - Limelight.convertAngleToDistance(points.get(1).distance));

        return new Trajectory(original.shooterRpm + adjustment, original.hoodPos);
    }

    public void add(double distance, Trajectory trajectory) {
        points.add(new Point(distance, trajectory));

        points.sort((Point p1, Point p2) -> {
            if (p1.distance < p2.distance) return -1;
            if (p1.distance > p2.distance) return 1;
            return 0;
        });
    }

    @Override
    public String toString() {
        return String.join(";", points.stream().map(Point::toString).toArray(String[]::new));
    }

    //method that converts key and returns array of maps based off following schema:
    //distance=0,shooterRpm=100,hoodPos=1;distance=2,shooterRpm=0,hoodPos=-1
    //for each instance, add to array
    // public static ArrayList<Map<String,Double>> convertKey(String key) {
    //     ArrayList<Map<String,Double>> maps = new ArrayList<>();
    //     String[] split = key.split(";");
    //     for(String s : split) {
    //         String[] split2 = s.split(",");
    //         Map<String,Double> map = new java.util.HashMap<>();
    //         for(String s2 : split2) {
    //             String[] split3 = s2.split("=");
    //             map.put(split3[0], Double.parseDouble(split3[1]));
    //         }
    //         maps.add(map);
    //     }
    //     return maps;        
    // }

    // public void updateMap() {
    //     String key = "distance=1.986525535583496,shooterRpm=3500,hoodPos=0.7";
    //     ArrayList<Map<String,Double>> maps = convertKey(key);
    //     double avgDistance = 0;
    //     double avgShooterRpm = 0;
    //     double avgHoodPos = 0;
    //     int total = 0;
    //     for(Map<String,Double> map : maps) {
    //         avgDistance += map.get("distance");
    //         avgShooterRpm += map.get("shooterRpm");
    //         avgHoodPos += map.get("hoodPos");
    //         total++;
    //     }
    //     avgDistance /= total;
    //     avgShooterRpm /= total;
    //     avgHoodPos /= total;

    //     try {
    //         Trajectory t = get(avgDistance);
    //         double deltaHoodPos = avgHoodPos / t.hoodPos;
    //         double deltaShooterRpm = avgShooterRpm / t.shooterRpm;
            
    //         points.replaceAll( i -> {
    //             double newShooterRpm = i.trajectory.shooterRpm * deltaShooterRpm;
    //             double newHoodPos = i.trajectory.hoodPos * deltaHoodPos;
    //             return new Point(i.distance, new Trajectory(Math.copySign(Math.min(Math.abs(newShooterRpm), 5000), newShooterRpm), Math.copySign(Math.min(Math.abs(newHoodPos), 1.0), newHoodPos)));
    //         });
    //     } catch (OutOfBoundsException e) {
    //         e.printStackTrace();
    //     }
        
    // }
}
