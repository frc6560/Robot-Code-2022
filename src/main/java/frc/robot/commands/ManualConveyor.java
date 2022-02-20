// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotIds;
import frc.robot.subsystems.Conveyor;

public class ManualConveyor extends CommandBase {
  public static interface Controls {
    boolean getConveyorMotor();
    boolean getBallChainReverse();
    boolean isIntakeEngaged();
  }

  private final Conveyor conveyor;
  private final Controls controls;

  private final NetworkTable ntTable;
  private NetworkTableEntry ntConveyorSpeed;
  private NetworkTableEntry ntTargetOverHead;
  private double conveyorSpeed;
  private double overHeadSpeed;
  

  /** Creates a new ConveyorCommand2. */
  public ManualConveyor(Conveyor conveyor, Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(conveyor);
    this.conveyor = conveyor;
    this.controls = controls;

    ntTable = NetworkTableInstance.getDefault().getTable("Intake");

    ntConveyorSpeed = ntTable.getEntry("Conveyor Speed");
    ntConveyorSpeed.setDouble(0.3);

    ntTargetOverHead = ntTable.getEntry("Over-head Speed");
    ntTargetOverHead.setDouble(0.3);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyor.setConveyor(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conveyorSpeed = ntConveyorSpeed.getDouble(0.0);
    overHeadSpeed = ntTargetOverHead.getDouble(0.0);

    if (controls.getBallChainReverse()){
      conveyor.setConveyor(-conveyorSpeed);
      conveyor.setOverHead(-overHeadSpeed);
    } else if (controls.getConveyorMotor() || controls.isIntakeEngaged()){
      if(!conveyor.getSensor()){
        conveyor.setConveyor(conveyorSpeed);
        conveyor.setOverHead(overHeadSpeed);
      }else{
        conveyor.setConveyor(0.0);
        conveyor.setOverHead(0.0);
      }
    } else {
      conveyor.setConveyor(0.0);
      conveyor.setOverHead(0.0);
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
