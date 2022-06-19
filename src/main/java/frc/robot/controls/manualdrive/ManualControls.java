// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls.manualdrive;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.ClimbCommand;

import frc.robot.utility.NumberStepper;
import frc.robot.utility.PovNumberStepper;
import edu.wpi.first.wpilibj.Joystick;

import static frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;

import static frc.robot.Constants.*;


/** Add your docs here. */
public class ManualControls implements DriveCommand.Controls, IntakeCommand.Controls, ShooterCommand.Controls, Limelight.Controls, ConveyorCommand.Controls, ClimbCommand.Controls {

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
    public boolean getIntakeOut() {
        return controlStation.getRawButton(ControllerIds.DRIVER_STATION_BUTTON_1); //|| controlStation.getRawButton(ControllerIds.DRIVER_STATION_TOGGLE_1);
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
    public boolean getFeedShooter() {
        return xbox.getRawAxis(ControllerIds.XBOX_R_TRIGGER) > 0.1;
    }

    @Override
    public boolean getBallChainReverse(){
        return controlStation.getRawButton(ControllerIds.DRIVER_STATION_BUTTON_2);
    }


    @Override
    public int getLimelightPipeline(){
        return controlStation.getRawButton(ControllerIds.DRIVER_STATION_TOGGLE_3) ? 2 : 2;
    }

    @Override
    public double shooterTurretTest(){
        return controlStation.getRawAxis(ControllerIds.DRIVER_STATION_X_AXIS);
    }

    @Override
    public double shooterHoodTest(){
        double value = controlStation.getRawAxis(ControllerIds.DRIVER_STATION_Y_AXIS);
        if(Math.abs(value) < 0.1){
            return 0.0;
        }
        return value;
    }

    @Override
    public boolean getAimShooter(){
        return controlStation.getRawButton(ControllerIds.DRIVER_STATION_TOGGLE_2);
    }

    @Override
    public boolean getConstantAiming() {
        return controlStation.getRawButton(ControllerIds.DRIVER_STATION_TOGGLE_3);
    }

    @Override
    public boolean overrideTurretCenter(){
        return xbox.getRawAxis(ControllerIds.XBOX_L_TRIGGER) > 0.1;
    }

    @Override
    public boolean getHotRPMAddition(){
        return xbox.getRawButton(ControllerIds.XBOX_A_BUTTON);
    }

    @Override
    public boolean getHotRPMReduction(){
        return xbox.getRawButton(ControllerIds.XBOX_B_BUTTON);
    }
    
    @Override
    public boolean getClimbRotatorEngaged() {
        return controlStation.getRawButton(ControllerIds.DRIVER_STATION_TOGGLE_4);
    }

    @Override
    public double getClimbExtensionMotors() {
        return controlStation.getRawAxis(ControllerIds.DRIVER_STATION_Y_AXIS);
    }

    @Override
    public boolean getAutoClimbEnabled(){
        return controlStation.getRawButton(ControllerIds.DRIVER_STATION_TOGGLE_1);
    }


    @Override
    public double driveGetX() {
        return xbox.getRawAxis(ControllerIds.XBOX_L_JOY_X);
    }


    @Override
    public double driveGetY() {
        return xbox.getRawAxis(ControllerIds.XBOX_L_JOY_Y);
    }


    @Override
    public double driveGetRotation() {
        return xbox.getRawAxis(ControllerIds.XBOX_R_JOY_X);
    }
}
