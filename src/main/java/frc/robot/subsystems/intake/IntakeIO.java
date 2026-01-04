// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeLeftConnected = false;
    public AngularVelocity leftVelocityRotPerSec = RevolutionsPerSecond.zero();
    public Current leftCurrentAmps = Amp.zero();

    public boolean intakeRightConnected = false;
    public AngularVelocity rightVelocityRotPerSec = RevolutionsPerSecond.zero();
    public Current rightCurrentAmps = Amp.zero();

    public boolean rotatorConnected = false;
    public Angle rotatorPositionRot = Degree.zero();
    public double rotatorClosedLoopErrorRot = 0.0;
    public AngularVelocity rotatorVelocityRotPerSec = RevolutionsPerSecond.zero();
  }

  //List of methods that each IO Layer should be accounting for
  /** Update the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Add motors to CTRE orchestra if available. */
  public default void addInstruments(Orchestra orchestra) {}

  /* Set the intake rotator angle.
   * Make sure units of measurement are consistent.
   * The Boathook example in this branch uses Radians to measure angle, while this intake uses degrees.
   * Choose what makes sense and keep it consistent throughout.
  */
  public default void setPosition(Angle angle) {}
  
  /** Set the intake contact wheel velocity. */
  public default void setIntakeSpeed(AngularVelocity velocity) {}

  /** Apply a neutral static brake to the intake rotator motor. */
  public default void setStaticBrake() {}

}