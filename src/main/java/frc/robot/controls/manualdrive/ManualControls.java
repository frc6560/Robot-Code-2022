// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls.manualdrive;

import frc.robot.commands.DriveCommand;

import frc.robot.utility.NumberStepper;
import frc.robot.utility.PovNumberStepper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;

import static frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;

import static frc.robot.Constants.*;


/** Add your docs here. */
public class ManualControls implements DriveCommand.Controls {

    private final Joystick xbox;
    private final Joystick controlStation;

    private final PovNumberStepper speed;
    private final PovNumberStepper turnSpeed;
    

    public ManualControls(Joystick xbox, Joystick controlStation) {
        this.xbox = xbox;
        this.controlStation = controlStation;

        this.speed = new PovNumberStepper(
            new NumberStepper(0.5, 0.2, PhysicalConstants.MAX_SPEED, 0.1),
            xbox,
            PovNumberStepper.PovDirection.VERTICAL
        );

        this.turnSpeed = new PovNumberStepper(
            new NumberStepper(0.5, 0.2, PhysicalConstants.MAX_TURN_SPEED, 0.05),
            xbox,
            PovNumberStepper.PovDirection.HORIZONTAL
        );

        
    }


    @Override
    public double driveGetX() {
        double val = xbox.getRawAxis(ControllerIds.XBOX_L_JOY_X);
        return Math.copySign(Math.pow(val,2), val);
    }


    @Override
    public double driveGetY() {
        double val = xbox.getRawAxis(ControllerIds.XBOX_L_JOY_Y);
        return Math.copySign(Math.pow(val,1), val);    
    }


    @Override
    public double driveGetRotation() {
        double val = xbox.getRawAxis(ControllerIds.XBOX_R_JOY_X);
        return Math.copySign(Math.pow(val,2), val);    
    }

    @Override
    public Rotation2d driveGetRotationPosition() {
        double rad = Math.min(1, Math.max(-1, driveGetRotation())) * Math.PI;
        return new Rotation2d(rad);
    }
}
