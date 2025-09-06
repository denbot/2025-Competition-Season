package frc.robot.commands.boathookCommands.setpointCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BoathookConstants;
import frc.robot.subsystems.boathook.Boathook;

public class setBoathookStateCommand extends Command {

  public enum boathookAngle {
    startAngle,
    setupAngle,
    scoreAngle,
    idleAngle,
    stabAngle,
    noChangeAngle;
  }

  public enum boathookLength {
    startLength,
    setupLength,
    scoreLength,
    idleLength,
    stabLength,
    noChangeLength;
  }

  boathookAngle angle;
  boathookLength length;

  Boathook boathook;

  boolean isAngleFinished = false;
  boolean isExtendFinished = false;

  public setBoathookStateCommand(Boathook boathook, boathookAngle angle, boathookLength length) {
    this.boathook = boathook;
    this.angle = angle;
    this.length = length;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    // both switches follow start -> setup -> score -> idle -> stab -> noChange
    // both also do set(Angle/Length) then update the isFinished boolean

    // angle dependent command
    switch (this.angle) {
      case startAngle -> {
        boathook.setAngle(boathook.getLevel().startAngle);
        isAngleFinished =
            Math.abs(
                    boathook.getAngle()
                        - (boathook.getLevel().startAngle + boathook.microRotationOffset))
                < 5;
      }
      case setupAngle -> {
        boathook.setAngle(boathook.getLevel().setupAngle);
        isAngleFinished =
            Math.abs(
                    boathook.getAngle()
                        - (boathook.getLevel().setupAngle + boathook.microRotationOffset))
                < 5;
      }
      case scoreAngle -> {
        boathook.setAngle(boathook.getLevel().scoreAngle);
        isAngleFinished =
            Math.abs(
                    boathook.getAngle()
                        - (boathook.getLevel().scoreAngle + boathook.microRotationOffset))
                < 5;
      }
      case idleAngle -> {
        boathook.setAngle(BoathookConstants.IDLE_ANGLE);
        isAngleFinished =
            Math.abs(
                    boathook.getAngle()
                        - (BoathookConstants.IDLE_ANGLE + boathook.microRotationOffset))
                < 5;
      }
      case stabAngle -> {
        boathook.setAngle(BoathookConstants.STAB_ANGLE);
        isAngleFinished =
            Math.abs(
                    boathook.getAngle()
                        - (BoathookConstants.STAB_ANGLE + boathook.microRotationOffset))
                < 5;
      }
      case noChangeAngle -> boathook.setAngle(boathook.getAngle()); // do nothing
    }

    // length dependent command
    switch (this.length) {
      case startLength -> {
        boathook.setLength(boathook.getLevel().startLength);
        isExtendFinished = Math.abs(boathook.getLength() - boathook.getLevel().startLength) < 0.1;
      }
      case setupLength -> {
        boathook.setLength(boathook.getLevel().setupLength);
        isExtendFinished = Math.abs(boathook.getLength() - boathook.getLevel().setupLength) < 0.1;
      }
      case scoreLength -> {
        boathook.setLength(boathook.getLevel().scoreLength);
        isExtendFinished = Math.abs(boathook.getLength() - boathook.getLevel().scoreLength) < 0.1;
      }
      case idleLength -> {
        boathook.setLength(BoathookConstants.IDLE_EXTENSION);
        isExtendFinished = Math.abs(boathook.getLength() - BoathookConstants.IDLE_EXTENSION) < 0.1;
      }
      case stabLength -> {
        boathook.setLength(BoathookConstants.STAB_EXTENSION);
        isExtendFinished = Math.abs(boathook.getLength() - BoathookConstants.STAB_ANGLE) < 0.1;
      }
      case noChangeLength -> boathook.setLength(boathook.getLength()); // do nothing
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return (isAngleFinished || this.angle == boathookAngle.noChangeAngle)
        && (isExtendFinished || this.length == boathookLength.noChangeLength);
  }
}
