package frc.robot.commands;

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
  TargetChange currentChange;
  boolean commandsRun = false;

  public AutoCommandScheduler(TargetChange... targetChanges) {
    this.targetChanges = targetChanges;
    this.currentChange = targetChanges[0];
  }

  /**
   * Function to itterate over all targetChange commands passed into the auto alignment. Also calls
   * an OTF command in order to cary out said alignment
   */
  public void runAutoCommands() {
    if (!commandsRun
        && (this.currentTargetIndex == 0
            || Robot.robotContainer.onTheFlyAlignCommand.pathFindingCommand.isFinished())) {

      if (this.currentTargetIndex >= this.targetChanges.length) {
        this.commandsRun = true;
        return;
      } else {
        System.out.println("Started New Step Of Auto");
        this.targetChanges[this.currentTargetIndex].schedule();
        Robot.robotContainer.onTheFlyAlignCommand.schedule();
        this.currentTargetIndex++;
      }
    }
  }
}
