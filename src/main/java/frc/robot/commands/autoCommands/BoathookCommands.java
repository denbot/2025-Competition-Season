package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.boathook.Boathook;
import java.util.function.BooleanSupplier;

public class BoathookCommands {

  private double startLength;
  private double itterations = 0;
  private int clockCycles = 80;
  private double gain = 15;
  private double offset = -0.1;
  private double scaling = 0.1;
  private Boathook boathook;
  private final LEDSubsystem ledSubsystem;

  public BoathookCommands(
      Boathook boathook,
      LEDSubsystem ledSubsystem
  ) {
    this.boathook = boathook;
    this.ledSubsystem = ledSubsystem;
  }

  public Command extendL2() {
    return new SequentialCommandGroup(
        setLengthLinearCommand(0.2),
        setAngleCommand(90),
        setLengthLinearCommand(1.1),
        setAngleCommand(120));
  }

  public Command retractL2() {
    return new SequentialCommandGroup(
        setLengthLinearCommand(0.93),
        setAngleCommand(144),
        setLengthLinearCommand(0.2),
        setAngleCommand(93));
  }

  public Command scoreL2() {
    return setCommandName(
        new SequentialCommandGroup(extendL2(), new WaitCommand(2), retractL2()), "Score_L2");
  }

  public Command extendL3() {
    return new SequentialCommandGroup(
        setLengthLinearCommand(0.2),
        setAngleCommand(90),
        setLengthLinearCommand(2.4),
        setAngleCommand(108));
  }

  public Command retractL3() {
    return new SequentialCommandGroup(
        setLengthLinearCommand(2.2),
        setAngleCommand(125),
        setLengthLinearCommand(0.2),
        setAngleCommand(93));
  }

  public Command scoreL3() {
    return setCommandName(
        new SequentialCommandGroup(extendL3(), new WaitCommand(2), retractL3()), "Score_L3");
  }

  public Command extendL4() {
    return new SequentialCommandGroup(
        setLengthLinearCommand(0.2),
        setAngleCommand(92),
        setLengthCurveCommand(4.6),
        setAngleCommand(97));
  }

  public Command retractL4() {
    return new SequentialCommandGroup(
        setLengthLinearCommand(2.45), setAngleCommand(93), setLengthLinearCommand(0.2));
  }

  public Command scoreL4() {
    return setCommandName(
        new SequentialCommandGroup(extendL4(), new WaitCommand(2), retractL4()), "Score_L4");
  }

  public Command setBoathookIdle() {
    return new SequentialCommandGroup(setAngleCommand(93), setLengthLinearCommand(0.2));
  }

  public Command setBoathookStab() {
    return new SequentialCommandGroup(setLengthLinearCommand(0.2), setAngleCommand(45));
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

  // exists because setName returns void,
  private Command setCommandName(Command command, String name) {
    command.setName(name);
    return command;
  }

  public Command handoffCommand(IntakeCommands intakeCommands) {

    return new SequentialCommandGroup(
        setAngleCommand(93),
        setLengthLinearCommand(0.06),
        setAngleCommand(25),
        intakeCommands.intakeSpearCommand(),
        new ParallelCommandGroup(setAngleCommand(93), intakeCommands.runRejectCommand()),
        intakeCommands.intakeL1Command(),
        ledSubsystem.fill(Color.kYellow)
    );
  }

  public Command setAngleCommand(double angle) {
    return Commands.run(() -> boathook.setAngle(angle))
        .alongWith(ledSubsystem.fill(ledSubsystem.rightBuffer, Color.kOrange))
        .until(isAngleFinished())
        .andThen(ledSubsystem.fill(ledSubsystem.rightBuffer, Color.kYellow));
  }

  public Command setLengthLinearCommand(double length) {
    return Commands.run(() -> boathook.setLength(length))
        .alongWith(ledSubsystem.fill(ledSubsystem.leftBuffer, Color.kOrange))
        .until(isLinearExtendFinished())
        .andThen(ledSubsystem.fill(ledSubsystem.leftBuffer, Color.kYellow));
  }

  public Command setLengthCurveCommand(double length) {
    return Commands.runOnce(
            () -> {
              startLength = boathook.getLength();
              itterations = 0;
              System.out.println(startLength + ", " + length);
            })
        .andThen(
            Commands.run(
                    () -> {
                      double setPoint =
                          startLength
                              + ((length - startLength)
                                  / (1
                                      + Math.exp(
                                          -gain
                                              * (itterations / 100
                                                  - offset
                                                  - scaling * (length - startLength)))));
                      System.out.println(itterations + " Stepoint: " + setPoint);
                      SmartDashboard.putNumber("Boathook Setpoint", setPoint);
                      itterations++;
                      boathook.setLength(setPoint);
                    })
                .alongWith(ledSubsystem.fill(ledSubsystem.leftBuffer, Color.kYellow))
                .until(isCurveExtendFinished()))
        .andThen(
            Commands.runOnce(() -> boathook.setLength(length))
                .andThen(ledSubsystem.fill(ledSubsystem.leftBuffer, Color.kOrange))
        );
  }

  public BooleanSupplier isLinearExtendFinished() {
    return () -> (Math.abs(boathook.getLengthSetpoint() - boathook.getLength()) < 0.1);
  }

  public BooleanSupplier isCurveExtendFinished() {
    return () ->
        (Math.abs(boathook.getLengthSetpoint() - boathook.getLength()) < 0.1
            && itterations > clockCycles);
  }

  public BooleanSupplier isAngleFinished() {
    return () -> (Math.abs(boathook.getAngleSetpoint() - boathook.getAngle()) < 5);
  }
}
