package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.commands.boathookCommands.SetLevelCommand;

public class AutoScoreCommand extends Command {

  private SetLevelCommand level;
  private static waitCommand scoreWaitCommand = new waitCommand(100);

  public AutoScoreCommand(SetLevelCommand level) {
    this.level = level;
  }

  // isReal() checks ensure that the command doesent get stuck in sim mode because
  // the sim has no simulation of the boathooks

  @Override
  public void initialize() {
    System.out.println("Starting Wait");
    if (this.level != null) this.level.schedule();
    if(!RobotBase.isReal()) scoreWaitCommand.schedule();
    else Robot.robotContainer.extendBoathook.schedule();
  }

  @Override
  public void execute() {
    if(RobotBase.isReal()){
        if (Robot.robotContainer.extendBoathook.isFinished()) scoreWaitCommand.schedule();
        if (scoreWaitCommand.isFinished()) Robot.robotContainer.retractBoathook.schedule();
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Ending Wait");
  }

  @Override
  public boolean isFinished() {
    if(RobotBase.isReal()) return Robot.robotContainer.retractBoathook.isFinished();
    return scoreWaitCommand.isFinished() || this.level == null;
  }
}
