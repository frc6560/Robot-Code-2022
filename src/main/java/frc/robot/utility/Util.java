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
}
