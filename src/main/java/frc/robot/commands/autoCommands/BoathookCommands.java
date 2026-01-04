package frc.robot.commands.autoCommands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.led.LEDController;
import frc.robot.subsystems.boathook.Boathook;

import static edu.wpi.first.units.Units.*;

public class BoathookCommands {
  private final Boathook boathook;
  private final LEDController ledController;

  public BoathookCommands(
      Boathook boathook,
      LEDController ledController
  ) {
    this.boathook = boathook;
    this.ledController = ledController;
  }

  public Command extendL2() {
    return new SequentialCommandGroup(
        setLengthLinearCommand(Inches.of(2.5)),
        setAngleCommand(Degrees.of(90)),
        setLengthLinearCommand(Inches.of(13)),
        setAngleCommand(Degrees.of(120)));
  }

  public Command retractL2() {
    return new SequentialCommandGroup(
        setLengthLinearCommand(Inches.of(11)),
        setAngleCommand(Degrees.of(144)),
        setLengthLinearCommand(Inches.of(2.5)),
        setAngleCommand(Degrees.of(93)));
  }

  public Command scoreL2() {
    return setCommandName(
        new SequentialCommandGroup(extendL2(), new WaitCommand(2), retractL2()), "Score_L2");
  }

  public Command extendL3() {
    return new SequentialCommandGroup(
        setLengthLinearCommand(Inches.of(2.5)),
        setAngleCommand(Degrees.of(90)),
        setLengthLinearCommand(Inches.of(28.5)),
        setAngleCommand(Degrees.of(108)));
  }

  public Command retractL3() {
    return new SequentialCommandGroup(
        setLengthLinearCommand(Inches.of(26)),
        setAngleCommand(Degrees.of(125)),
        setLengthLinearCommand(Inches.of(2.5)),
        setAngleCommand(Degrees.of(93)));
  }

  public Command scoreL3() {
    return setCommandName(
        new SequentialCommandGroup(extendL3(), new WaitCommand(2), retractL3()), "Score_L3");
  }

  public Command extendL4() {
    return new SequentialCommandGroup(
        setLengthLinearCommand(Inches.of(2.5)),
        setAngleCommand(Degrees.of(92)),
        setLengthCurveCommand(Inches.of(54.5)),
        setAngleCommand(Degrees.of(97)));
  }

  public Command retractL4() {
    return new SequentialCommandGroup(
        setLengthLinearCommand(Inches.of(29)),
        setAngleCommand(Degrees.of(93)),
        setLengthLinearCommand(Inches.of(2.5))
    );
  }

  public Command scoreL4() {
    return setCommandName(
        new SequentialCommandGroup(extendL4(), new WaitCommand(2), retractL4()), "Score_L4");
  }

  public Command setBoathookIdle() {
    return new SequentialCommandGroup(
        setAngleCommand(Degrees.of(93)),
        setLengthLinearCommand(Inches.of(2.5))
    );
  }

  public Command setBoathookStab() {
    return new SequentialCommandGroup(
        setLengthLinearCommand(Inches.of(2.5)),
        setAngleCommand(Degrees.of(45))
    );
  }

  public Command microAdjustExtensionForward() {
    return Commands.runOnce(() -> boathook.setLength(boathook.getLengthSetpoint().plus(Inch.of(0.5))));
  }

  public Command microAdjustExtensionBackward() {
    return Commands.runOnce(() -> boathook.setLength(boathook.getLengthSetpoint().minus(Inch.of(0.5))));
  }

  public Command microAdjustAngleForward() {
    return Commands.runOnce(() -> boathook.setAngle(boathook.getAngleSetpoint().plus(Degrees.of(7))));
  }

  public Command microAdjustAngleBackward() {
    return Commands.runOnce(() -> boathook.setAngle(boathook.getAngleSetpoint().minus(Degrees.of(7))));
  }

  // exists because setName returns void,
  private Command setCommandName(Command command, String name) {
    command.setName(name);
    return command;
  }

  public Command handoffCommand(Intake intake) {

    return new SequentialCommandGroup(
        setAngleCommand(Degrees.of(93)),
        setLengthLinearCommand(Inches.of(1)),
        setAngleCommand(Degrees.of(25)),
        intakeCommands.intakeSpearCommand(),
        new ParallelCommandGroup(setAngleCommand(Degrees.of(93)), intakeCommands.runRejectCommand()),
        intakeCommands.intakeL1Command(),
        ledController.fill(Color.kYellow)
    );
  }

  public Command setAngleCommand(Angle angle) {
    return Commands.run(() -> boathook.setAngle(angle))
        .alongWith(ledController.fill(ledController.rightBuffer, Color.kOrange))
        .until(this::isAngleFinished)
        .andThen(ledController.fill(ledController.rightBuffer, Color.kYellow));
  }

  public boolean isAngleFinished() {
    return boathook.getAngleSetpoint().minus(boathook.getAngle()).abs(Degrees) < 5;
  }

  public Command setLengthLinearCommand(Distance length) {
    return Commands.run(() -> boathook.setLength(length))
        .alongWith(ledController.fill(ledController.leftBuffer, Color.kOrange))
        .until(this::isLinearExtendFinished)
        .andThen(ledController.fill(ledController.leftBuffer, Color.kYellow));
  }

  public boolean isLinearExtendFinished() {
    return boathook.getLengthSetpoint().minus(boathook.getLength()).abs(Meters) < 0.1;
  }

  public Command setLengthCurveCommand(Distance targetLength) {
    /*
    The original version of this command was using the revolution units. Since we're just using this to teach, the hope
    is that using meters as the explanation would make the function "close enough".
     */
    var sCurveBoathook = new Command() {
      private double startLength;
      private double iterations = 0;

      @SuppressWarnings("FieldCanBeLocal")
      private final int clockCycles = 80;
      @SuppressWarnings("FieldCanBeLocal")
      private final double gain = 15;
      @SuppressWarnings("FieldCanBeLocal")
      private final double offset = -0.1;
      @SuppressWarnings("FieldCanBeLocal")
      private final double scaling = 0.1;

      @Override
      public void initialize() {
        startLength = boathook.getLength().in(Meters);
        iterations = 0;
        System.out.println(startLength + ", " + targetLength);
      }

      @Override
      public void execute() {
        double targetDiff = targetLength.in(Meters) - startLength;
        double setPoint = startLength + (targetDiff / (1 + Math.exp(-gain * (iterations / 100 - offset - scaling * targetDiff))));
        System.out.println(iterations + " Stepoint: " + setPoint);
        SmartDashboard.putNumber("Boathook Setpoint", setPoint);
        iterations++;
        boathook.setLength(Meters.of(setPoint));
      }

      @Override
      public boolean isFinished() {
        return boathook.getLengthSetpoint().minus(boathook.getLength()).abs(Meters) < 0.1 && iterations > clockCycles;
      }
    };

    return sCurveBoathook
        .alongWith(ledController.fill(ledController.leftBuffer, Color.kYellow))
        .andThen(
            Commands.runOnce(() -> boathook.setLength(targetLength))
                .andThen(ledController.fill(ledController.leftBuffer, Color.kOrange))
        );
  }
}
