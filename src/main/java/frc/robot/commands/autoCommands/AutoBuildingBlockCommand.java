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
    System.out.println("New BB With: " + autoAlignment + ", " + scorelevel + "\n");
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
    if (RobotBase.isReal()) {
      if (currentStep == 0
          && (this.autoAlignment == null
              || Robot.robotContainer.onTheFlyAlignCommand.pathFindingCommand.isFinished())) {
        Robot.robotContainer.extendBoathook.schedule();
        currentStep++;
      }
      if (currentStep == 1 && Robot.robotContainer.extendBoathook.isFinished()) {
        scoreWaitCommand.schedule();
        currentStep++;
      }
      if (currentStep == 2 && scoreWaitCommand.isFinished()) {
        Robot.robotContainer.retractBoathook.schedule();
        currentStep++;
      }
    } else {
      if (Robot.robotContainer.onTheFlyAlignCommand.pathFindingCommand.isFinished()) {
        if (!scoreWaitCommand.isScheduled()) scoreWaitCommand.schedule();
      } else scoreWaitCommand.cancel();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if(scoreWaitCommand.isScheduled()) scoreWaitCommand.cancel();
    System.out.println("Ended Auto Building Block CMD");
    currentStep = 0;
  }

  @Override
  public boolean isFinished() {
    return Robot.robotContainer.retractBoathook.isFinished()
        || (!RobotBase.isReal() && scoreWaitCommand.isFinished());
  }
}
