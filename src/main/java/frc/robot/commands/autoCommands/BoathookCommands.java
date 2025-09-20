package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.boathook.Boathook;
import java.util.function.BooleanSupplier;

public class BoathookCommands {

  private Boathook boathook;

  public BoathookCommands(Boathook boathook) {
    this.boathook = boathook;
  }

  public Command extendL2() {
    System.out.println("Extending L2");
    return new SequentialCommandGroup(
        setLengthCommand(0.2), setAngleCommand(90), setLengthCommand(1.1), setAngleCommand(120));
  }

  public Command retractL2() {
    System.out.println("Retracting L2");
    return new SequentialCommandGroup(
        setLengthCommand(0.93), setAngleCommand(144), setLengthCommand(0.2), setAngleCommand(93));
  }

  public Command scoreL2() {
    return new SequentialCommandGroup(extendL2(), new WaitCommand(2), retractL2());
  }

  public Command extendL3() {
    System.out.println("Extending L3");
    return new SequentialCommandGroup(
        setLengthCommand(0.2), setAngleCommand(90), setLengthCommand(2.4), setAngleCommand(108));
  }

  public Command retractL3() {
    System.out.println("Retracting L3");
    return new SequentialCommandGroup(
        setLengthCommand(2.2), setAngleCommand(125), setLengthCommand(0.2), setAngleCommand(93));
  }

  public Command scoreL3() {
    return new SequentialCommandGroup(extendL3(), new WaitCommand(2), retractL3());
  }

  public Command extendL4() {
    System.out.println("Extending L4");
    return new SequentialCommandGroup(
        setLengthCommand(0.2), setAngleCommand(90), setLengthCommand(4.6), setAngleCommand(97));
  }

  public Command retractL4() {
    System.out.println("Retracting L4");
    return new SequentialCommandGroup(
        setLengthCommand(2.45), setAngleCommand(93), setLengthCommand(0.2));
  }

  public Command scoreL4() {
    return new SequentialCommandGroup(extendL4(), new WaitCommand(2), retractL4());
  }

  public Command setBoathookIdle() {
    System.out.println("Setting Boathook Idle");
    return new SequentialCommandGroup(setAngleCommand(93), setLengthCommand(0.2));
  }

  public Command setBoathookStab() {
    System.out.println("Setting Boathook Stab");
    return new SequentialCommandGroup(setLengthCommand(0.2), setAngleCommand(35));
  }

  public Command MicroAdjustExtensionForward() {
    return Commands.runOnce(() -> boathook.setLength(boathook.getLengthSetpoint() + 0.01));
  }

  public Command MicroAdjustExtensionBackward() {
    return Commands.runOnce(() -> boathook.setLength(boathook.getLengthSetpoint() - 0.01));
  }

  public Command MicroAdjustAngleForward() {
    return Commands.runOnce(() -> boathook.setAngle(boathook.getAngleSetpoint() + 0.01));
  }

  public Command MicroAdjustAngleBackward() {
    return Commands.runOnce(() -> boathook.setAngle(boathook.getAngleSetpoint() - 0.01));
  }

  public Command handoffCommand(IntakeCommands intakeCommands) {
    return new SequentialCommandGroup(
        setAngleCommand(93),
        setLengthCommand(0.06),
        setAngleCommand(25),
        intakeCommands.intakeSpearCommand(),
        new ParallelCommandGroup(setAngleCommand(93), intakeCommands.intakeL1Command()));
  }

  public Command setAngleCommand(double angle) {
    System.out.println("Setting Angle To: " + angle);
    return (Commands.run(() -> boathook.setAngle(angle))).until(isAngleFinished());
  }

  public Command setLengthCommand(double length) {
    System.out.println("Setting Length To: " + length);
    return (Commands.run(() -> boathook.setLength(length))).until(isExtendFinished());
  }

  public BooleanSupplier isExtendFinished() {
    System.out.println("Length" + boathook.getLengthSetpoint() + ", " + boathook.getLength());
    return () -> (Math.abs(boathook.getLengthSetpoint() - boathook.getLength()) < 0.1);
  }

  public BooleanSupplier isAngleFinished() {
    System.out.println("Angle" + boathook.getAngleSetpoint() + ", " + boathook.getAngle());
    return () -> (Math.abs(boathook.getAngleSetpoint() - boathook.getAngle()) < 5);
  }
}
