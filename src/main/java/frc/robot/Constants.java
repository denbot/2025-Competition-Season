// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum Direction { // Implemented so that we don't have to do booleans and inverses
    LEFT,
    RIGHT
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final String canivoreSerial = "D75CCE723353385320202034111303FF";
  }

  public static class IntakeConstants {
    public static final double rotatorGearRatio = 1;
    public static final double forwardSoftLimit = 0.25;
    public static final double reverseSoftLimit = 0;

    public static final int LEFT_INTAKE_MOTOR_ID = 19;
    public static final int RIGHT_INTAKE_MOTOR_ID = 20;

    public static final int CANDI_ID = 18; // S1 is first switch, S2 is base
    public static final int INTAKE_ROTATION_MOTOR_ID = 23;
    public static final int INTAKE_ROTATION_ENCODER_ID = 24;

    public static final double intakeDownAngle = 0;
    public static final double intakeSpearAngle = 0.55;
    public static final double intakeL1Angle = 0.2;

    public static final double intakeSpeed = 60; // Rotations / Second
    public static final double intakeAcceleration = 13; // Rotations / Second^2
  }

  public static class BoathookConstants {
    public static final int EXTENDER_MOTOR_ID = 13;
    public static final int ROTATION_MOTOR_ID = 14;
    public static final int ROTATION_ENCODER_ID = 15;
    public static final int EXTENDER_ENCODER_ID = 16;
    public static final int CANDI_ID = 17; // S1 is rotation, S2 is extension
    public static final double ROTATOR_GEAR_RATIO = 25;
    public static final double ROTATOR_FORWARD_LIMIT = 0.34;
    public static final double ROTATOR_REVERSE_LIMIT = 0.0;
    //    public static final double EXTENDER_CANCODER_RATIO = 101.6 / 40;
    //    public static final double EXTENDER_GEAR_RATIO = 25;
    // 4 inch mechanism drum
    // 40 mm sensor
    public static final double EXTENDER_FORWARD_LIMIT = 4.55; // not in inches (rotations)
    public static final double EXTENDER_REVERSE_LIMIT = 0;

    // Rotation in degrees
    public static final double STAB_ANGLE = 32;
    public static final double IDLE_ANGLE = 90;
    public static final double L2_SETUP_ANGLE = 115;
    public static final double L3_SETUP_ANGLE = 108;
    public static final double L4_SETUP_ANGLE = 97;
    public static final double L2_SCORE_ANGLE = 133;
    public static final double L3_SCORE_ANGLE = 115;
    public static final double L4_SCORE_ANGLE = 97;

    // Extension in rotations
    public static final double STAB_EXTENSION = 0;
    public static final double IDLE_EXTENSION = 0;
    public static final double L2_EXTENSION = 0.75;
    public static final double L3_EXTENSION = 1.95;
    public static final double L4_EXTENSION = 4.2;
  }
}
