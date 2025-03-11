package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
    intake.setIntakeSpeed(direction.speed);
    intake.setAngle(Constants.IntakeConstants.intakeDownAngle, 0);
    SmartDashboard.putString("IntakeStatus", "Running");
  }

  @Override
  public boolean isFinished() {
    if (direction == Direction.Intake) {
      return intake.isCoralIntaken();
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
