// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotIds;
import frc.robot.commands.autonomous.AutonomousController;
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

  private final NetworkTable ntTable;

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

    ntTable = NetworkTableInstance.getDefault().getTable("Transfer");
  }

  public ConveyorCommand(Conveyor conveyor, Shooter shooter, Boolean shoot){ // Autonomouse
    this(conveyor, new AutonomousController(false,"conveyor", (shoot ? "shooter" : "")), shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyor.setConveyor(0);
    conveyorTopSensorLast = conveyor.getSensor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controls.getBallChainReverse()){
      conveyor.setConveyor(-conveyorSpeed);
      conveyor.setOverHead(-overHeadSpeed);

    } else if (controls.getConveyorMotor() || controls.isIntakeEngaged() || (controls.getFeedShooter() && shooter.isShooterReady())){  // if ballchain, intake, or shooter_feeding is on, run transfer

      if(!conveyor.getSensor() || (controls.getFeedShooter() && shooter.isShooterReady())){
        conveyor.setConveyor(conveyorSpeed);

      }else{
        conveyor.setConveyor(0.0);
      }

      conveyor.setOverHead(overHeadSpeed);

      if(conveyorTopSensorLast && !conveyor.getSensor()){
        shooter.increaseBallCount();
      }

    } else {
      conveyor.setConveyor(0.0);
      conveyor.setOverHead(0.0);
    }

    conveyorTopSensorLast = conveyor.getSensor();
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
