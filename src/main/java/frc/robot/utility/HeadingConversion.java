// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utility;

/** Add your docs here. */
public class HeadingConversion {
    private double previousGyroAngle;
    private double totalGyroAngle;
    private double currentGyroAngle;


    public HeadingConversion() {
        this.currentGyroAngle = 0.0;
        this.totalGyroAngle = 0.0;
        this.previousGyroAngle = 0.0;
    }

    public double getTotalHeading(double currentGyroAngle) {
        this.totalGyroAngle += getHeadingDiff(currentGyroAngle, previousGyroAngle);
        this.previousGyroAngle = this.currentGyroAngle;

        return this.totalGyroAngle;
    }

    public static double getHeadingDiff(double h1, double h2) {
        double left = h1 - h2;
        double right = h2 - h1;
        if (left < 0) left += 360.0;
        if (right < 0) right += 360.0;
        return left < right ? -left : right;
    }

}