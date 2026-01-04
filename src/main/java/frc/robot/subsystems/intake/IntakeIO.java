// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double leftVelocityRevPerSec = 0.0;
    public double leftCurrentAmps = 0.0;

    public double rightVelocityRevPerSec = 0.0;
    public double rightCurrentAmps = 0.0;

    public double rotatorPositionDeg = 0.0;
    public double rotatorClosedLoopErrorDeg = 0.0;
    public double rotatorVelocityRevPerSec = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setAngle(double angle) {}

  public default void setIntakeSpeed(double velocity) {}

  public default void setStaticBrake() {}

}