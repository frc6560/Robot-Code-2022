// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.commands.ManualDrive;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.ManualShooter;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.ManualConveyor;
import frc.robot.commands.ManualClimb;

import frc.robot.utility.NumberStepper;
import frc.robot.utility.PovNumberStepper;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;

import java.util.HashMap;

import static frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;


import static frc.robot.Constants.*;


/** Add your docs here. */
public class AutonomousController implements ManualIntake.Controls, ManualShooter.Controls, Limelight.Controls, ManualConveyor.Controls {

    private HashMap<String, Boolean> subsystemMap;

    private boolean useLimelightFar = false;

    public AutonomousController(boolean useLimelightFar, String... subsystems) {
      subsystemMap = new HashMap<String, Boolean>() {{
        put("intake", false);
        put("shooter", false);
        put("conveyor", false);
      }};

      for (String sub : subsystems) {
        sub = sub.toLowerCase();
        subsystemMap.replace(sub, true);
      }

    }

    @Override
    public boolean getIntakeOut() {
        return subsystemMap.get("intake");
    }

    @Override
    public boolean isIntakeEngaged(){
        return getIntakeOut();
    }

    @Override
    public boolean getConveyorMotor() {
        return subsystemMap.get("conveyor");
    }

    @Override
    public boolean getFeedShooter() {
      return subsystemMap.get("shooter");
    }

    @Override
    public boolean getBallChainReverse(){
        return false;
    }


    @Override
    public int getLimelightPipeline(){
        if (getAimShooter())
          return useLimelightFar ? 2 : 1;
        return 0;
    }

    @Override
    public boolean getAimShooter(){
        return subsystemMap.get("shooter");
    }


    @Override
    public double shooterHoodTest() {
      // TODO Auto-generated method stub
      return 0;
    }


    @Override
    public double shooterTurretTest() {
      // TODO Auto-generated method stub
      return 0;
    }

}
