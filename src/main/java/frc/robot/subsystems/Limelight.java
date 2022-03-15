// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import static frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public static interface Controls {
    int getLimelightPipeline();
  }
    
  private final NetworkTableEntry ntX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
  private final NetworkTableEntry ntY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
  private final NetworkTableEntry ntV = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
  private final NetworkTableEntry ntPipeline = NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline");

  private final Controls controls;
  private boolean forceOff = true;

  public Limelight(Controls controls) {
    this.controls = controls;
    setForceOff(false);

    ntDispTab("Limelight")
    .add("Distance",this::getDistance)
    .add("Horizontal Angle", this::getHorizontalAngle)
    .add("Vertical Angle", this::getVertAngle)
    ;
  }

   /**
   * 
   * @return distance to target from center of robot **IN FEET**!!!!
   */
  // public double getDistance() {
  //   double limelightDist = ((104 - 24) / Math.tan(Math.toRadians(52.0 + ntY.getDouble(0.0)))) / 12;
  //   double multiplyer = 1;
  //   if(limelightDist > 15){ // 22- 15
  //     multiplyer = 1.489572989076465;
  //   }
  //   if(limelightDist > 10){ // 15 - 10
  //     multiplyer = 1.392757660167131;
  //   }
  //   if(limelightDist > 5){ // 10 - 5
  //     multiplyer = 1.453488372093023;
  //   }
  //   if(limelightDist > 0){ // 5 - 0
  //     multiplyer = 1.453488372093023;
  //   }
  //   return  limelightDist * multiplyer;//TODO: FIX THIS LATER!!!!!
  // }

  public double getDistance() {
    // return -0.376023*ntY.getDouble(0.0) + 11.0681;
    return ntY.getDouble(0.0);
  }

  public double getHorizontalAngle() {
    return ntX.getDouble(0.0);
  }

  public double getVertAngle() {
    return ntY.getDouble(0.0);
  }

  public boolean hasTarget(){
    return ntV.getDouble(0.0) == 1.0;
  }

  public void setForceOff(boolean value) {
    forceOff = value;
  }

  @Override
  public void periodic() {
    ntPipeline.setNumber(forceOff ? 0 : controls.getLimelightPipeline());
  }
}
