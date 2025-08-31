package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.commands.boathookCommands.SetLevelCommand;
import frc.robot.commands.visionCommands.TargetChange;

public class AutoBuildingBlockCommand extends Command {
  private TargetChange autoAlignment;
  private SetLevelCommand scoreLevel;
  private waitCommand scoreWaitCommand;
  private int currentStep = 0;

  public AutoBuildingBlockCommand(TargetChange autoAlignment, SetLevelCommand scorelevel) {
    this.autoAlignment = autoAlignment;
    this.scoreLevel = scorelevel;
    this.scoreWaitCommand = new waitCommand(100);
  }

  @Override
  public void initialize() {
    currentStep = 0;
    System.out.println("Started New Building Block CMD");
    if (this.autoAlignment != null) {
      this.autoAlignment.schedule();
      Robot.robotContainer.onTheFlyAlignCommand.schedule();
    }
    if (this.scoreLevel != null)
      scoreLevel.schedule(); // only sets the level, doesent rely on real boathooks
  }

  @Override
  public void execute() {
    // extendMotionPath and retractMotionPath rely on physical sensors not present in the sim
    // in the sim, these commands never finish, thus they arent run in the sim
    if (RobotBase.isReal()) {

      // If the robot has stopped moving, start the extension
      if (currentStep == 0
          && (this.autoAlignment == null
              || Robot.robotContainer.onTheFlyAlignCommand.pathFindingCommand.isFinished())) {
        Robot.robotContainer.extendBoathook.schedule();
        currentStep++;
      }
      // if the extention has finished, wait for a couple seconds
      if (currentStep == 1 && Robot.robotContainer.extendBoathook.isFinished()) {
        scoreWaitCommand.schedule();
        currentStep++;
      }
      // if the wait is finished, retract the boathooks and scrore
      if (currentStep == 2 && scoreWaitCommand.isFinished()) {
        Robot.robotContainer.retractBoathook.schedule();
        currentStep++;
      }
      // if the robot is being simulated, simulate a score by scheduling a wait command only when
      // the movement is finished
    } else {
      if (Robot.robotContainer.onTheFlyAlignCommand.pathFindingCommand.isFinished()) {
        if (!scoreWaitCommand.isScheduled()) scoreWaitCommand.schedule();
      } else scoreWaitCommand.cancel();
    }
  }

  // ensure no wait commands are running in the middle of other auto alignments
  @Override
  public void end(boolean interrupted) {
    if (scoreWaitCommand.isScheduled()) scoreWaitCommand.cancel();
    System.out.println("Ended Auto Building Block CMD");
    currentStep = 0;
  }

  // if the boathooks are retracted or the wait command is finished and it is a sim
  @Override
  public boolean isFinished() {
    return Robot.robotContainer.retractBoathook.isFinished()
        || (!RobotBase.isReal() && scoreWaitCommand.isFinished());
  }
}
