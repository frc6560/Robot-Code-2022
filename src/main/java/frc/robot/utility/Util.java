package frc.robot.utility;

import edu.wpi.first.math.geometry.Rotation2d;

public class Util {
    public static double getLimited(double num, double maxMagnitude) {
        if (num > maxMagnitude) {
            return maxMagnitude;
        } else if (num < -maxMagnitude) {
            return -maxMagnitude;
        } else {
            return num;
        }
    }
    
    public static double getHeadingDiff(double h1, double h2) {
        double left = h1 - h2;
        double right = h2 - h1;
        if (left < 0) left += 360.0;
        if (right < 0) right += 360.0;
        return left < right ? -left : right;
    }

    public static Rotation2d getHeadingDiff(Rotation2d h1, Rotation2d h2) {
        Rotation2d left = h1.minus(h2);
        Rotation2d right = h2.minus(h1);
        if (left.getDegrees() < 0) left.plus(new Rotation2d(2*Math.PI));
        if (right.getDegrees() < 0) right.plus(new Rotation2d(2*Math.PI));
        return left.getDegrees() < right.getDegrees() ? left.unaryMinus() : right;
    }
}
