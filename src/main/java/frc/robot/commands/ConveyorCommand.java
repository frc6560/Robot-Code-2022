// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class ConveyorCommand extends CommandBase {
  public static interface Controls {
    boolean getConveyorMotor();
    boolean getBallChainReverse();
    boolean isIntakeEngaged();
    boolean getFeedShooter();
  }

  private final Conveyor conveyor;
  private final Controls controls;
  private final Shooter shooter;

  private final double conveyorSpeed = 0.45;
  private final double overHeadSpeed = 0.7;

  private boolean conveyorTopSensorLast = false;
  

  /** Creates a new ConveyorCommand2. */
  public ConveyorCommand(Conveyor conveyor, Controls controls, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(conveyor);
    this.conveyor = conveyor;
    this.controls = controls;

    this.shooter = shooter; // this is to be able to read wether or not the shooter is ready to shoot.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyor.setConveyor(0);
    conveyorTopSensorLast = conveyor.getSensor();
  }

  public enum ConveyorState {
    REVERSE, INTAKE, SHOOT, CHILLING
  }

  public synchronized void setConveyorState(ConveyorState state) {
    switch (state) {
      case REVERSE:
        conveyor.setConveyor(-conveyorSpeed);
        conveyor.setOverHead(-overHeadSpeed);
        break;
      case INTAKE:
        if (!conveyor.getSensor())
          conveyor.setOverHead(overHeadSpeed);

        conveyor.setConveyor(conveyorSpeed * (shooter.isShooterReady() ? 1.5 : 1));

        if(conveyorTopSensorLast && !conveyor.getSensor()){
          shooter.increaseBallCount();
        }

        break;
      case SHOOT:
        conveyor.setOverHead(overHeadSpeed);
        if (shooter.isShooterReady())
          conveyor.setConveyor(conveyorSpeed * (shooter.isShooterReady() ? 1.5 : 1));

        if(conveyorTopSensorLast && !conveyor.getSensor()){
          shooter.increaseBallCount();
        }

        break;
      case CHILLING:
        conveyor.setConveyor(0.0);
        conveyor.setOverHead(0.0);
        break;
    }
    conveyorTopSensorLast = conveyor.getSensor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controls.getBallChainReverse()){
      setConveyorState(ConveyorState.REVERSE);
    } else if (controls.getConveyorMotor() || controls.isIntakeEngaged()){  // if ballchain, intake, or shooter_feeding is on, run transfer

      setConveyorState(ConveyorState.INTAKE);

    } else if (controls.getFeedShooter() || controls.getBallChainReverse()){
      setConveyorState(ConveyorState.SHOOT);

    } else {
      setConveyorState(ConveyorState.CHILLING);
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.setConveyor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
