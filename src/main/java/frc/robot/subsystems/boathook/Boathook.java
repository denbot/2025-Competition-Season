// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.boathook;

import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CanBeAnInstrument;

import static edu.wpi.first.units.Units.*;

/** Creates a new Boathook. */
public class Boathook extends SubsystemBase implements CanBeAnInstrument {

  private final BoathookIO io;
  private final BoathookIOInputsAutoLogged inputs = new BoathookIOInputsAutoLogged();
  private Angle angleSetpoint = Radian.zero();
  private Distance lengthSetpoint = Meter.zero();

  public Boathook(BoathookIO io) {
    this.io = io;
  }

  public void setAngle(Angle angle) {
    io.setAnglePosition(angle);
    angleSetpoint = angle;
  }

  public Angle getAngle() {
    return inputs.angle;
  }

  public Angle getAngleSetpoint() {
    return angleSetpoint;
  }

  public void setLength(Distance length) {
    io.setLengthPosition(length);
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

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Boathook/Angle", getAngle().in(Degrees));
    SmartDashboard.putNumber("Boathook/Extension", getLength().in(Meters));
  }
}
