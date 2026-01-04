// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static final double rotatorGearRatio = 1;
  public static final double forwardSoftLimit = 0.25;
  public static final double reverseSoftLimit = 0;

  public static final int LEFT_INTAKE_MOTOR_ID = 19;
  public static final int RIGHT_INTAKE_MOTOR_ID = 20;

  public static final int CANDI_ID = 18; // S1 is unused, S2 is base switch
  public static final int INTAKE_ROTATION_MOTOR_ID = 23;
  public static final int INTAKE_ROTATION_ENCODER_ID = 24;

  public static final double intakeDownAngle = 0;
  public static final double intakeSpearAngle = 0.55;
  public static final double intakeL1Angle = 0.2;

  public static final double intakeSpeed = 60; // Rotations / Second
  public static final double intakeAcceleration = 13; // Rotations / Second^2
}