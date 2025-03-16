package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.intake.Intake;

public class RunIntakeCommand extends Command {
  public enum Direction {
    Intake(Constants.IntakeConstants.intakeSpeed),
    Eject(-Constants.IntakeConstants.intakeSpeed);

    private final double speed;

    Direction(double speed) {
      this.speed = speed;
    }
  }

  private final Intake intake;
  private final Direction direction;
  private final Timer ejectingWait = new Timer();

  public RunIntakeCommand(Intake intake, Direction direction) {
    this.intake = intake;
    this.direction = direction;
    addRequirements(this.intake);
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {
    ejectingWait.reset();
    SmartDashboard.putString("IntakeStatus", "Running");

    if (direction == Direction.Intake && intake.isCoralIntaken()) {
      return; // Don't even start moving, we already have a piece
    }

    intake.setIntakeSpeed(direction.speed);
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("Coral status", intake.isCoralIntaken());
  }

  @Override
  public boolean isFinished() {
    if (direction == Direction.Intake) {
      boolean coralIntaken = intake.isCoralIntaken();

      if (coralIntaken) {
        Robot.rumble().coralIntaken.schedule();
      }

      return coralIntaken;
    }

    if (direction == Direction.Eject) {
      if (intake.isCoralIntaken()) {
        return false; // Still on the nub, need to wait for it not to be
      }

      if (!ejectingWait.isRunning()) {
        ejectingWait.restart();
        return false;
      }

      if (ejectingWait.hasElapsed(.4)) {
        Robot.rumble().coralEjected.schedule();
        return true;
      }
    }

    return false; // Cancels based on button released
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    SmartDashboard.putString("IntakeStatus", "Stopped");
  }
}
