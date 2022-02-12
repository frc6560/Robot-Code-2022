/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utility;

/**
 * Add your docs here.
 */
public class NumberStepper {
    private final double min;
    private final double max;
    private final double step;

    private double current;

    public NumberStepper(double initial, double min, double max, double step) {
        this.min = min;
        this.max = max;
        this.step = step;

        current = initial;
    }
    public NumberStepper(double min, double max, double step) {
        this(min, min, max, step);
    }

    public double get() {
        return current;
    }

    public void set(double num) {
        current = num;
        limit();
    }

    public void stepUp() {
        current += step;
        limit();
    }

    public void stepDown() {
        current -= step;
        limit();
    }

    private void limit() {
        current = getClamped(current, min, max);
    }

    private static double getClamped(double num, double min, double max) {
        if (num > max) {
            return max;
        } else if (num < min) {
            return min;
        } else {
            return num;
        }
    }
}
