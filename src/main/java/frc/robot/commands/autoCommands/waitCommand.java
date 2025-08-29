package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;

public class waitCommand extends Command {

  private int delay;
  private int curTimeWaited = 0;

  public waitCommand(int delay) {
    this.delay = delay;
  }

  @Override
  public void initialize() {
    this.curTimeWaited = 0;
  }

  @Override
  public void execute() {
    this.curTimeWaited++;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return this.curTimeWaited > this.delay;
  }
}
