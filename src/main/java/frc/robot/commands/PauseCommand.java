package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class PauseCommand extends Command {
  private final Timer timer;
  private final double timeToPause;
  private final Drive drive;

  public PauseCommand(Drive driveSubsystem, double timeToPause) {
    this.drive = driveSubsystem;
    timer = new Timer();
    this.timeToPause = timeToPause;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    this.drive.stop();
    timer.restart();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(timeToPause);
  }
}
