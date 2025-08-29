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
    if (this.level != null) this.level.schedule(); // level = null when it is a placeholder / formatting command

    if (!RobotBase.isReal()) scoreWaitCommand.schedule(); // if the robot is in sim mode, only schedule wait
    else Robot.robotContainer.extendBoathook.schedule(); // else, extend the boathooks
}

  @Override
  public void execute() {
    // extend and retract have no meaning in the sim and as such only cause the command to get stuck in sim mode
    if (RobotBase.isReal()) {
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
    // similarly, we only care about delay in the sim, not the boathooks as they arent simulated
    if (RobotBase.isReal()) return Robot.robotContainer.retractBoathook.isFinished();
    return scoreWaitCommand.isFinished() || this.level == null; // wait is always finished if it is a placeholder commnand
  }
}
