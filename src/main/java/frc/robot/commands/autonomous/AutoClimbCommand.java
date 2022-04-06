// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class AutoClimbCommand extends CommandBase {
  /** Creates a new AutoClimbCommand. */
  private final Climb climb;

  int autoClimbPhase = 0;
  int cycles = 0;
  double pauseTime = -1;
  
  private final double UPPER_HEIGHT = 23;
  private final double LOWER_HEIGHT = 0.025;
  private final double WAIT_DURATION = 0.5;

  double extensionSpeed = 1;

  public AutoClimbCommand(Climb climb) {
    addRequirements(climb);

    this.climb = climb;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // climb.setExtensionMotors(0);
    // climb.setRotatorPiston(false);
    autoClimbPhase = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // climb.setExtensionMotors(0.1);

    if(autoClimbPhase == 0){
      climb.setExtensionMotors(0.5);

      if(climb.getLeftPositionInches() > 4){
        climb.setRotatorPiston(true);
      }

      if(climb.getLeftPositionInches() > UPPER_HEIGHT - (cycles == 0 ? 2 : 0) || climb.getRightPositionInches() > UPPER_HEIGHT - (cycles == 0 ? 2 : 0)){
        autoClimbPhase ++;
        
        System.out.println("\n\n\n\n1: Phase: " + autoClimbPhase);
      }
    } else if(autoClimbPhase == 1){
      climb.setExtensionMotors(0);
      climb.setRotatorPiston(false);

      if(pauseTime == -1){
        pauseTime = Timer.getFPGATimestamp();
      }
      double curTime = Timer.getFPGATimestamp();

      if(curTime - pauseTime > WAIT_DURATION){
        pauseTime = -1;
        autoClimbPhase++;
        
        System.out.println("\n\n\n\n3: Phase: " + autoClimbPhase);
      }
    } else if(autoClimbPhase == 2){
      climb.setRotatorPiston(false);
      climb.setExtensionMotors(-extensionSpeed);
      
      if(climb.getLeftPositionInches() < LOWER_HEIGHT || climb.getRightPositionInches() < LOWER_HEIGHT){
        autoClimbPhase ++;
        
        System.out.println("\n\n\n\n2: Phase: " + autoClimbPhase);
      }
    } else if(autoClimbPhase == 3){
      climb.setExtensionMotors(0);

      if(pauseTime == -1){
        pauseTime = Timer.getFPGATimestamp();
      }
      double curTime = Timer.getFPGATimestamp();

      if(curTime - pauseTime > WAIT_DURATION){
        pauseTime = -1;
        autoClimbPhase++;
        cycles++;
        
        System.out.println("\n\n\n\n3: Phase: " + autoClimbPhase);
      }
    }

    autoClimbPhase %= 4;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    initialize();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
