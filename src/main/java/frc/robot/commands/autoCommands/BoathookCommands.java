package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.boathook.Boathook;
import java.util.function.BooleanSupplier;

public class BoathookCommands {

  private Boathook boathook;

  public BoathookCommands(Boathook boathook) {
    this.boathook = boathook;
  }

  public Command extendL2() {
    return new SequentialCommandGroup(
        setLengthCommand(0.2), setAngleCommand(90), setLengthCommand(1.1), setAngleCommand(120));
  }

  public Command retractL2() {
    return new SequentialCommandGroup(
        setLengthCommand(0.93), setAngleCommand(144), setLengthCommand(0.2), setAngleCommand(93));
  }

  public Command scoreL2() {
    return setCommandName(
        new SequentialCommandGroup(extendL2(), new WaitCommand(2), retractL2()), "Score_L2");
  }

  public Command extendL3() {
    return new SequentialCommandGroup(
        setLengthCommand(0.2), setAngleCommand(90), setLengthCommand(2.4), setAngleCommand(108));
  }

  public Command retractL3() {
    return new SequentialCommandGroup(
        setLengthCommand(2.2), setAngleCommand(125), setLengthCommand(0.2), setAngleCommand(93));
  }

  public Command scoreL3() {
    return setCommandName(
        new SequentialCommandGroup(extendL3(), new WaitCommand(2), retractL3()), "Score_L3");
  }

  public Command extendL4() {
    return new SequentialCommandGroup(
        setLengthCommand(0.2), setAngleCommand(90), setLengthCommand(4.6), setAngleCommand(97));
  }

  public Command retractL4() {
    return new SequentialCommandGroup(
        setLengthCommand(2.45), setAngleCommand(93), setLengthCommand(0.2));
  }

  public Command scoreL4() {
    return setCommandName(
        new SequentialCommandGroup(extendL4(), new WaitCommand(2), retractL4()), "Score_L4");
  }

  public Command setBoathookIdle() {
    return new SequentialCommandGroup(setAngleCommand(93), setLengthCommand(0.2));
  }

  public Command setBoathookStab() {
    return new SequentialCommandGroup(setLengthCommand(0.2), setAngleCommand(45));
  }

  public Command MicroAdjustExtensionForward() {
    return Commands.runOnce(() -> boathook.setLength(boathook.getLengthSetpoint() + 0.05));
  }

  public Command MicroAdjustExtensionBackward() {
    return Commands.runOnce(() -> boathook.setLength(boathook.getLengthSetpoint() - 0.05));
  }

  public Command MicroAdjustAngleForward() {
    return Commands.runOnce(() -> boathook.setAngle(boathook.getAngleSetpoint() + 0.05));
  }

  public Command MicroAdjustAngleBackward() {
    return Commands.runOnce(() -> boathook.setAngle(boathook.getAngleSetpoint() - 0.05));
  }

  private Command setCommandName(Command command, String name) {
    command.setName(name);
    return command;
  }

  public Command handoffCommand(IntakeCommands intakeCommands, Leds led) {
    return new SequentialCommandGroup(
        setAngleCommand(93),
        setLengthCommand(0.08),
        setAngleCommand(25),
        intakeCommands.intakeSpearCommand(),
        new ParallelCommandGroup(setAngleCommand(93), intakeCommands.intakeL1Command()),
        Commands.runOnce(() -> led.solidInSection(0, 21, 60, 255, 255)));
  }

  public Command setAngleCommand(double angle) {
    return (Commands.run(
            () -> {
              boathook.setAngle(angle);
              Robot.robotContainer.leds.solidInSectionRight(30, 255, 255);
            }))
        .until(isAngleFinished())
        .andThen(
            Commands.runOnce(() -> Robot.robotContainer.leds.solidInSectionRight(60, 255, 255)));
  }

  public Command setLengthCommand(double length) {
    return (Commands.run(
            () -> {
              boathook.setLength(length);
              Robot.robotContainer.leds.solidInSectionLeft(30, 255, 255);
            }))
        .until(isExtendFinished())
        .andThen(
            Commands.runOnce(() -> Robot.robotContainer.leds.solidInSectionLeft(60, 255, 255)));
  }

  public BooleanSupplier isExtendFinished() {
    return () -> (Math.abs(boathook.getLengthSetpoint() - boathook.getLength()) < 0.1);
  }

  public BooleanSupplier isAngleFinished() {
    return () -> (Math.abs(boathook.getAngleSetpoint() - boathook.getAngle()) < 5);
  }
}
