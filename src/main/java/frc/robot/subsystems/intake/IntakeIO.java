// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.Orchestra;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeLeftConnected = false;
    public double leftVelocityRevPerSec = 0.0;
    public double leftCurrentAmps = 0.0;

    public boolean intakeRightConnected = false;
    public double rightVelocityRevPerSec = 0.0;
    public double rightCurrentAmps = 0.0;

    public boolean rotatorConnected = false;
    public double rotatorPositionDeg = 0.0;
    public double rotatorClosedLoopErrorDeg = 0.0;
    public double rotatorVelocityRevPerSec = 0.0;
  }

  //List of methods that each IO Layer should be accounting for
  /** Update the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Add motors to CTRE orchestra if available. */
  public default void addInstruments(Orchestra orchestra) {}

  /** Set the intake rotator angle. */
  public default void setAngle(double angle) {}
  
  /** Set the intake contact wheel velocity. */
  public default void setIntakeSpeed(double velocity) {}

  /** Apply a neutral static brake to the intake rotator motor. */
  public default void setStaticBrake() {}

}