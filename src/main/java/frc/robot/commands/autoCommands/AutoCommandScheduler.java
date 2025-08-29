package frc.robot.commands.autoCommands;

import frc.robot.Robot;
import frc.robot.commands.visionCommands.TargetChange;

/**
 * Specific Command Scheduler for auto alignment sequences. Pathplanner OTF commands call commands
 * in their execute and finish before the command they initialized finishes. Due to this, the
 * default Sequential Command Group runs into scheduling conflicts and does not work.
 */
public class AutoCommandScheduler {

  TargetChange[] targetChanges;
  int currentTargetIndex = 0;

  AutoScoreCommand[] scoreCommands;
  int currentScoreIndex = 0;
  AutoScoreCommand currentScore;

  private boolean currentCommandIsTarget = true;

  boolean commandsRun = false;

  public AutoCommandScheduler(TargetChange[] targetChanges, AutoScoreCommand[] scoreCommands) {
    // if it is passed in empty arrays, do nothing
    if(targetChanges.length == 0 || scoreCommands.length == 0) this.commandsRun = true;
    else {
      this.targetChanges = targetChanges;

      this.scoreCommands = scoreCommands;
      this.currentScore = scoreCommands[0];
    }
  }

  /**
   * Function to itterate over all targetChange commands passed into the auto alignment. Also calls
   * an OTF command in order to cary out said alignment
   */
  public void runAutoCommands() {
    if (!commandsRun) {
      if(this.currentScoreIndex >= this.scoreCommands.length && this.currentTargetIndex >= this.targetChanges.length) {
        commandsRun = true;
        System.out.println("Finished Auto Routine");
      }
      
      // Prevent IndexOutOfBounds errors, 
      // If it is not the initial alignment, 
      // if the current command running is an alignment, making the next command a score command
      // if that given alignment is completed

      if (this.currentScoreIndex < this.scoreCommands.length 
          && this.currentTargetIndex != 0
          && currentCommandIsTarget
          && Robot.robotContainer.onTheFlyAlignCommand.pathFindingCommand.isFinished() == true) {

        System.out.println("Calling New Score Command");
        currentCommandIsTarget = false; // current running command will no longer be an alignment
        this.scoreCommands[this.currentScoreIndex].schedule(); // schedule scoring command
        this.currentScore = scoreCommands[this.currentScoreIndex]; // update current score variable
        this.currentScoreIndex++; // incriment score index
        System.out.println("\n");
      }

      // Prevent IndexOutOfBounds errors, 
      // If it is the initial alignment, 
      // if the current command running is a score command, making the next command a score command
      // if that given score is completed
      
      if (this.currentTargetIndex < this.targetChanges.length 
          && (this.currentTargetIndex == 0
          || (!currentCommandIsTarget && this.currentScore.isFinished() == true))) {
        System.out.println("Called New OTF");
        currentCommandIsTarget = true;
         // schedule the commands to auto align
        this.targetChanges[this.currentTargetIndex].schedule();
        Robot.robotContainer.onTheFlyAlignCommand.schedule();
        this.currentTargetIndex++; // incriment index
        System.out.println("\n");
      }
    }
  }
}
