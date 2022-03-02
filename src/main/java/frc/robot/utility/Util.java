package frc.robot.utility;

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
}
