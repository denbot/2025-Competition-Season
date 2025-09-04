package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import frc.robot.commands.boathookCommands.HandoffCommand;
import frc.robot.subsystems.boathook.Boathook;
import frc.robot.subsystems.boathook.Boathook.boathookInfo;
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

  private final Boathook boathook;
  private final Intake intake;
  private final Direction direction;
  private final Timer runningWait = new Timer();

  private final IntakeMoveCommand liftToL1;
  private final HandoffCommand runHandoff;

  public RunIntakeCommand(Intake intake, Boathook boathook, Direction direction) {
    this.intake = intake;
    this.boathook = boathook;
    this.direction = direction;
    this.liftToL1 = new IntakeMoveCommand(intake, false, IntakeConstants.intakeL1Angle, 1, 2);
    this.runHandoff = new HandoffCommand(boathook, intake);
    addRequirements(this.intake);
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {
    runningWait.reset();
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
        if (!runningWait.isRunning()) {
          runningWait.restart();
          return false;
        }

        if (runningWait.hasElapsed(.2)) {
          if (boathook.getLevel() == boathookInfo.L1) {
            liftToL1.schedule();
            intake.flipL1Toggle();
          } else {
            runHandoff.schedule();
          }
          return true;
        }
      }
    }

    if (direction == Direction.Eject) {
      if (intake.isCoralIntaken()) {
        return false; // Still on the nub, need to wait for it not to be
      }

      if (!runningWait.isRunning()) {
        runningWait.restart();
        return false;
      }

      if (runningWait.hasElapsed(.4)) {
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
