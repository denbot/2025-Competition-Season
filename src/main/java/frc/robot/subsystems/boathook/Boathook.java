// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.boathook;

import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.CanBeAnInstrument;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LEDController;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

/** Creates a new this. */
public class Boathook extends SubsystemBase implements CanBeAnInstrument {

  private final BoathookIO io;
  private final LEDController ledController;
  private final BoathookIOInputsAutoLogged inputs = new BoathookIOInputsAutoLogged();
  private Angle angleSetpoint = Radian.zero();
  private Distance lengthSetpoint = Meter.zero();

  public Boathook(BoathookIO io, LEDController ledController) {
    this.io = io;
    this.ledController = ledController;
  }

  public void setAngle(Angle angle) {
    io.setAngle(angle);
    angleSetpoint = angle;
  }

  public Angle getAngle() {
    return inputs.angle;
  }

  public Angle getAngleSetpoint() {
    return angleSetpoint;
  }

  public void setLength(Distance length) {
    io.setLength(length);
    lengthSetpoint = length;
  }

  public Distance getLength() {
    return inputs.extensionLength;
  }

  public Distance getLengthSetpoint() {
    return lengthSetpoint;
  }

  public void addInstruments(Orchestra orchestra) {
    if(io instanceof CanBeAnInstrument instrument) {
      instrument.addInstruments(orchestra);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Boathook", inputs);

    Logger.recordOutput("Boathook/Angle", inputs.angle);
    Logger.recordOutput("Boathook/Angle Setpoint", angleSetpoint);
    Logger.recordOutput("Boathook/Extension", inputs.extensionLength);
    Logger.recordOutput("Boathook/Extension Setpoint", lengthSetpoint);
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
    return new SequentialCommandGroup(extendL2(), new WaitCommand(2), retractL2()).withName("Score_L2");
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
    return new SequentialCommandGroup(extendL3(), new WaitCommand(2), retractL3()).withName("Score_L3");
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
    return new SequentialCommandGroup(extendL4(), new WaitCommand(2), retractL4()).withName("Score_L4");
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
    return Commands.runOnce(() -> this.setLength(this.getLengthSetpoint().plus(Inch.of(0.5))));
  }

  public Command microAdjustExtensionBackward() {
    return Commands.runOnce(() -> this.setLength(this.getLengthSetpoint().minus(Inch.of(0.5))));
  }

  public Command microAdjustAngleForward() {
    return Commands.runOnce(() -> this.setAngle(this.getAngleSetpoint().plus(Degrees.of(7))));
  }

  public Command microAdjustAngleBackward() {
    return Commands.runOnce(() -> this.setAngle(this.getAngleSetpoint().minus(Degrees.of(7))));
  }

  public Command handoffCommand(Intake intake) {

    return new SequentialCommandGroup(
        new ConditionalCommand(
          setAngleCommand(Degrees.of(93)),
            Commands.none(),
            () -> this.getAngle().in(Degrees) > 93
        ),
        setLengthLinearCommand(Inches.of(1)),
        setAngleCommand(Degrees.of(25)),
        intake.intakeSpearCommand(),
        new ParallelCommandGroup(setAngleCommand(Degrees.of(93)), intake.runRejectCommand()),
        intake.intakeL1Command(),
        ledController.fill(Color.kYellow)
    );
  }

  private Command setAngleCommand(Angle angle) {
    return this.runOnce(() -> this.setAngle(angle))
        .alongWith(ledController.fill(ledController.rightBuffer, Color.kOrange))
        .andThen(Commands.waitUntil(this::isAngleFinished))
        .andThen(ledController.fill(ledController.rightBuffer, Color.kYellow));
  }

  private boolean isAngleFinished() {
    return this.getAngleSetpoint().minus(this.getAngle()).abs(Degrees) < 5;
  }

  private Command setLengthLinearCommand(Distance length) {
    return this.runOnce(() -> this.setLength(length))
        .alongWith(ledController.fill(ledController.leftBuffer, Color.kOrange))
        .andThen(Commands.waitUntil(this::isLinearExtendFinished))
        .andThen(ledController.fill(ledController.leftBuffer, Color.kYellow));
  }

  private boolean isLinearExtendFinished() {
    return this.getLengthSetpoint().minus(this.getLength()).abs(Meters) < 0.1;
  }

  private Command setLengthCurveCommand(Distance targetLength) {
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
        startLength = Boathook.this.getLength().in(Meters);
        iterations = 0;
      }

      @Override
      public void execute() {
        double targetDiff = targetLength.in(Meters) - startLength;
        double setPoint = startLength + (targetDiff / (1 + Math.exp(-gain * (iterations / 100 - offset - scaling * targetDiff))));
        SmartDashboard.putNumber("Boathook Setpoint", setPoint);
        iterations++;
        Boathook.this.setLength(Meters.of(setPoint));
      }

      @Override
      public boolean isFinished() {
        return Boathook.this.getLengthSetpoint().minus(Boathook.this.getLength()).abs(Meters) < 0.1 && iterations > clockCycles;
      }
    };

    return sCurveBoathook
        .alongWith(ledController.fill(ledController.leftBuffer, Color.kYellow))
        .andThen(
            Commands.runOnce(() -> this.setLength(targetLength))
                .andThen(ledController.fill(ledController.leftBuffer, Color.kOrange))
        );
  }
}
