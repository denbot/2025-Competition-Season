package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {

  private Intake intake;

  public IntakeCommands(Intake intake) {
    this.intake = intake;
  }

  public Command runIntakeCommand() {
    return Commands.runEnd(() -> intake.setIntakeSpeed(60), () -> intake.setIntakeSpeed(0));
  }

  public Command runRejectCommand() {
    return Commands.runEnd(() -> intake.setIntakeSpeed(-60), () -> intake.setIntakeSpeed(0))
        .raceWith(new WaitCommand(0.5));
  }

  public Command intakeDownCommand() {
    return Commands.run(() -> intake.setAngle(0, 0, -3))
        .until(() -> intake.getRotationVelocity() < 0.01);
  }

  public Command intakeL1Command() {
    return Commands.run(() -> intake.setAngle(0.2, 0, -3))
        .until(() -> intake.getRotationVelocity() < 0.01);
  }

  public Command intakeSpearCommand() {
    return Commands.run(() -> intake.setAngle(0.55, 0, -3))
        .until(() -> intake.getRotationVelocity() < 0.01);
  }
}
