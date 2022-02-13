// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.controls.manualdrive;

import frc.robot.commands.ManualDrive;
import frc.robot.commands.ManualShooter;
import frc.robot.utility.NumberStepper;
import frc.robot.utility.PovNumberStepper;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;

import static frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;


import static frc.robot.Constants.*;


/** Add your docs here. */
public class ManualControls implements ManualDrive.Controls, ManualShooter.Controls{

    private final Joystick xbox;
    private final PovNumberStepper speed;
    private final PovNumberStepper turnSpeed;


    public ManualControls(Joystick xbox, Joystick controlStation) {
        this.xbox = xbox;

        this.speed = new PovNumberStepper(
            new NumberStepper(0.5, 0.1, PhysicalConstants.MAX_SPEED, 0.1),
            xbox,
            PovNumberStepper.PovDirection.VERTICAL
        );

        this.turnSpeed = new PovNumberStepper(
            new NumberStepper(0.5, 0.1, PhysicalConstants.MAX_TURN_SPEED, 0.05),
            xbox,
            PovNumberStepper.PovDirection.HORIZONTAL
        );

        ntDispTab("Driver")
            .add("DEV: xbox X Position", this::getX)
            .add("DEV: xbox Y Position", this::getY);
    }

    @Override
    public double getX() {
        return xbox.getRawAxis(ControllerIds.XBOX_R_JOY_X);
    }

    @Override
    public double getY() {
        return -xbox.getRawAxis(ControllerIds.XBOX_L_JOY_Y);
    }

    @Override
    public double getSpeed() {
        return speed.get();
    }

    @Override
    public double getTurnSpeed() {
        return turnSpeed.get();
    }

    @Override
    public boolean isShooting() {
        return true; //TODO: Add button (probably control station)
    }

}
