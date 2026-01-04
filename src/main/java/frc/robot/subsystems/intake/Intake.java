// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CanBeAnInstrument;

public class Intake extends SubsystemBase implements CanBeAnInstrument {
  /** Creates a new Intake. */
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    SmartDashboard.putNumber("Intake Rotation Angle", inputs.rotatorPositionDeg);
  }

  public void addInstruments(Orchestra orchestra) {
    io.addInstruments(orchestra);
  }

  public Command runIntakeCommand() {
    return Commands.runEnd(() -> io.setIntakeSpeed(60), () -> io.setIntakeSpeed(0));
  }

  public Command runSoftIntakeCommand() {
    return Commands.runOnce(() -> io.setIntakeSpeed(5));
  }

  public Command runRejectCommand() {
    return Commands.runEnd(() -> io.setIntakeSpeed(-60), () -> io.setIntakeSpeed(0))
        .raceWith(new WaitCommand(0.5));
  }

  public Command intakeDownCommand() {
    return Commands.run(() -> io.setAngle(5))
        .until(() -> Math.abs(inputs.rotatorClosedLoopErrorDeg) < 1);
  }

  public Command intakeL1Command() {
    return Commands.run(() -> io.setAngle(72))
        .until(() -> Math.abs(inputs.rotatorClosedLoopErrorDeg) < 1);
  }

  public Command intakeSpearCommand() {
    return Commands.run(() -> io.setAngle(160))
        .until(
            () ->
                Math.abs(inputs.rotatorClosedLoopErrorDeg) < 20
                    && Math.abs(inputs.rotatorVelocityRevPerSec) < 1);
  }
}
