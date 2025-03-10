package frc.robot.commands.intakeCommands;

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
    // TODO Check sensors and cancel based on that and direction (intake expects sensors to trigger,
    // eject wants to see them stop)
    return false; // Cancels based on button released
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    SmartDashboard.putString("IntakeStatus", "Stopped");
  }
}
