/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utility;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Add your docs here.
 */
public class PovNumberStepper {
    public enum PovDirection {
        HORIZONTAL,
        VERTICAL
    }

    private final NumberStepper stepper;
    private final Joystick joystick;
    private final PovDirection direction;

    private int lastPov = -1;

    public PovNumberStepper(NumberStepper stepper, Joystick joystick, PovDirection direction) {
        this.stepper = stepper;
        this.joystick = joystick;
        this.direction = direction;

        CommandScheduler.getInstance().registerSubsystem(
            new Subsystem() {
                @Override
                public void periodic() {
                    update();
                }
            }
        );
    }

    private void update() {
        int pov = joystick.getPOV();

        if (lastPov == -1) {
            if (direction == PovDirection.VERTICAL) {
                if (pov == 0) {
                    stepper.stepUp();
                } else if (pov == 180) {
                    stepper.stepDown();
                }
            } else if (direction == PovDirection.HORIZONTAL) {
                if (pov == 90) {
                    stepper.stepUp();
                } else if (pov == 270) {
                    stepper.stepDown();
                }
            } else {
                throw new RuntimeException("unknown PovDirection value");
            }
        }

        lastPov = pov;
    }

    public double get() {
        return stepper.get();
    }

    public void set(double num) {
        stepper.set(num);
    }
}
