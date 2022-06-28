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
  private AutoClimbState state;

  int cycles = 0;
  double pauseTime = -1;
  
  private final double UPPER_HEIGHT = 23.25;
  private final double LOWER_HEIGHT = 0.025;
  private final double DOWN_WAIT_DURATION = 0.5;
  private final double UP_WAIT_DURATION = 0.25;

  private final double extensionSpeed = 1;

  public AutoClimbCommand(Climb climb) {
    addRequirements(climb);

    this.climb = climb;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = AutoClimbState.EXTENSION;
  }

  public enum AutoClimbState {
    EXTENSION, FIRST_TIMER, RETRACTION, SECOND_TIMER
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Phase 0: pisons out and extension
    // Phase 1: timer
    // Phase 2: pistons in and retraction
    // Phase 3: timer

    double leftPos = climb.getLeftPositionInches();
    double rightPos = climb.getRightPositionInches();
    double curTime = Timer.getFPGATimestamp();

    switch (state) {
      case EXTENSION:
        climb.setExtensionMotors(extensionSpeed);

        if(leftPos > 22){
          climb.setRotatorPiston(false);
        } else if(leftPos > 4){
          climb.setRotatorPiston(true);
        }

        if(leftPos > UPPER_HEIGHT - (cycles == 0 ? 4 : 0) || rightPos > UPPER_HEIGHT - (cycles == 0 ? 4 : 0)){
          state = AutoClimbState.FIRST_TIMER;
          System.out.println("\n\n\n\n1: Phase: " + state);
          break;
        }
        
        state = AutoClimbState.FIRST_TIMER;
        break;
      case FIRST_TIMER:
        climb.setExtensionMotors(0);
        climb.setRotatorPiston(false);

        if(pauseTime == -1){
          pauseTime = Timer.getFPGATimestamp();
        }
        

        if(curTime - pauseTime > DOWN_WAIT_DURATION){
          pauseTime = -1;
          state = AutoClimbState.RETRACTION;
        }

        break;
      case RETRACTION:
        climb.setRotatorPiston(false);
        climb.setExtensionMotors(-extensionSpeed);
        
        if(leftPos < LOWER_HEIGHT || rightPos < LOWER_HEIGHT){
          state = AutoClimbState.SECOND_TIMER;
          System.out.println("\n\n\n\n2: Phase: " + state);
        }
        break;
      case SECOND_TIMER:
        climb.setExtensionMotors(0);

        if(pauseTime == -1){
          pauseTime = Timer.getFPGATimestamp();
        }

        if(curTime - pauseTime > UP_WAIT_DURATION){
          pauseTime = -1;
          state = AutoClimbState.EXTENSION;
          cycles++;
          
          System.out.println("\n\n\n\n3: Phase: " + state);
        }
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
