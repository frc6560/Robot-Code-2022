// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.ConveyorCommand;

import java.util.HashMap;



/** Add your docs here. */
public class AutonomousController implements IntakeCommand.Controls, ShooterCommand.Controls, Limelight.Controls, ConveyorCommand.Controls {

    private HashMap<String, Boolean> subsystemMap;

    private boolean useLimelightFar;

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
      this.useLimelightFar = useLimelightFar;
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
    public boolean overrideTurretCenter(){
      return false;
    }
    

    @Override
    public boolean getHotRPMChange(){
        return false;
    }

    @Override
    public boolean getHotHoodChange(){
        return false;
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
