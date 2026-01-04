// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CanBeAnInstrument;

import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase implements CanBeAnInstrument {
  /** Creates a new Intake. */
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private double rotatorPositionSetpoint = 0.0;
  private double intakeVelocitySetpoint = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    SmartDashboard.putNumber("Intake/Rotator Position", getRotatorPosition().in(Degrees));
    SmartDashboard.putNumber("Intake/Rotator Setpoint", rotatorPositionSetpoint);
    SmartDashboard.putNumber("Intake/Intake Velocity", getIntakeVelocity().in(RotationsPerSecond));
    SmartDashboard.putNumber("Intake/Intake Velocity Setpoint", intakeVelocitySetpoint);
  }

  public void addInstruments(Orchestra orchestra) {
    io.addInstruments(orchestra);
  }

  public void setRotationAngle(Angle angle){
    io.setPosition(angle);
    rotatorPositionSetpoint = angle.in(Degrees);
  }

  public void setIntakeVelocity(AngularVelocity velocity){
    io.setIntakeSpeed(velocity);
    intakeVelocitySetpoint = velocity.in(RevolutionsPerSecond);
  }

  public Angle getRotatorPosition(){
    return inputs.rotatorPositionRot;
  }

  public AngularVelocity getRotatorVelocity(){
    return inputs.rotatorVelocityRotPerSec;
  }

  public AngularVelocity getIntakeVelocity(){
    return inputs.leftVelocityRotPerSec;
  }

  /* The boathook shows an example of commands being held in a separate class.
   * This subsystem holds its commands within the subsystem class for easy integration with our internal
   * state machine generator later.
  */

  public Command runIntakeCommand() {
    return Commands.runEnd(
      () -> setIntakeVelocity(RotationsPerSecond.of(60)), 
      () -> setIntakeVelocity(RotationsPerSecond.of(0))
    );
  }

  public Command runSoftIntakeCommand() {
    return Commands.runOnce(() -> setIntakeVelocity(RotationsPerSecond.of(5)));
  }

  public Command runRejectCommand() {
    return Commands.runEnd(
      () -> setIntakeVelocity(RotationsPerSecond.of(-60)),
      () -> setIntakeVelocity(RotationsPerSecond.of(0))
    ).raceWith(new WaitCommand(0.5));
  }

  public Command intakeDownCommand() {
    return Commands.run(() -> setRotationAngle(Degree.of(5)))
        .until(() -> Math.abs(Units.rotationsToDegrees(inputs.rotatorClosedLoopErrorRot)) < 1);
  }

  public Command intakeL1Command() {
    return Commands.run(() -> setRotationAngle(Degree.of(72)))
        .until(() -> Math.abs(Units.rotationsToDegrees(inputs.rotatorClosedLoopErrorRot)) < 1);
  }

  public Command intakeSpearCommand() {
    return Commands.run(() -> setRotationAngle(Degree.of(160)))
        .until(
            () ->
                Math.abs(Units.rotationsToDegrees(inputs.rotatorClosedLoopErrorRot)) < 20
                    && Math.abs(getRotatorVelocity().in(RotationsPerSecond)) < 1);
  }
}
