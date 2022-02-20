// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.controls.manualdrive;

import frc.robot.commands.ManualDrive;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.ManualShooter;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.ManualConveyor;
import frc.robot.commands.ManualClimb;

import frc.robot.utility.NumberStepper;
import frc.robot.utility.PovNumberStepper;
import edu.wpi.first.wpilibj.Joystick;

import static frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;


import static frc.robot.Constants.*;


/** Add your docs here. */
public class ManualControls implements ManualDrive.Controls, ManualIntake.Controls, ManualShooter.Controls, Limelight.Controls, ManualConveyor.Controls, ManualClimb.Controls {

    private final Joystick xbox;
    private final Joystick controlStation;
    
    private final Joystick xbox2;

    private final PovNumberStepper speed;
    private final PovNumberStepper turnSpeed;


    public ManualControls(Joystick xbox, Joystick controlStation, Joystick xbox2) {
        this.xbox = xbox;
        this.controlStation = controlStation;
        this.xbox2 = xbox2;

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
    public boolean getIntakeOut() {
        return controlStation.getRawButton(ControllerIds.DRIVER_STATION_BUTTON_1) || controlStation.getRawButton(ControllerIds.DRIVER_STATION_TOGGLE_1);
    }

    @Override
    public boolean isIntakeEngaged(){
        return getIntakeOut();
    }

    @Override
    public boolean getConveyorMotor() {
        return controlStation.getRawButton(ControllerIds.DRIVER_STATION_BUTTON_3);
    }

    @Override
    public boolean getBallChainReverse(){
        return controlStation.getRawButton(ControllerIds.DRIVER_STATION_BUTTON_2);
    }

    @Override
    public boolean isShooting() {
        //TODO: set actual button
        return xbox.getRawButton(2); // Button B, I believe (too lazy to check)
    }

    @Override
    public int getLimelightPipeline(){
        return controlStation.getRawButton(ControllerIds.DRIVER_STATION_TOGGLE_3) ? 1 : 0;
    }

    @Override
    public double shooterTurretTest(){
        return xbox2.getRawButton(ControllerIds.XBOX_R_BUMPER) ? 1 : xbox2.getRawButton(ControllerIds.XBOX_L_BUMPER) ? -1 : 0 ;
    }

    @Override
    public double shooterHoodTest(){
        return xbox2.getRawAxis(ControllerIds.XBOX_R_JOY_Y);
    }

    @Override
    public double shooterRPMTest(){
        return xbox2.getRawAxis(ControllerIds.XBOX_R_TRIGGER);
    }

    @Override
    public double getClimbRotatorSpeed() {
        return controlStation.getRawAxis(ControllerIds.DRIVER_STATION_Y_AXIS) * 1.0; //TODO: change 1.0 to whatever speed we want
    }

    @Override
    public double setClimbExtensionMotors() {
        return controlStation.getRawAxis(ControllerIds.DRIVER_STATION_X_AXIS) * 1.0; //TODO: change 1.0 to whatever speed we want
    }

    @Override
    public boolean setClimbPiston() {
        return controlStation.getRawButton(ControllerIds.DRIVER_STATION_TOGGLE_4);
    }
}
