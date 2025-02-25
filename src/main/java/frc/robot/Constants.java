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
    public static final double rotatorGearRatio = 90;
    public static final double forwardSoftLimit = 0.25;
    public static final double reverseSoftLimit = 0;

    public static final int INTAKE_MOTOR_ID = 21;

    public static final int INTAKE_ROTATION_MOTOR_ID = 23;
    public static final int INTAKE_ROTATION_ENCODER_ID = 24;

    public static final int INDEXER_LEFT_MOTOR_ID = 19;
    public static final int INDEXER_RIGHT_MOTOR_ID = 20;

    public static final double intakeDownAngle = 5;
    public static final double intakeFunnelAngle = 90;

    public static final double intakeSpeed = 4;
    public static final double indexerSpeed = 3;
  }

  public static class BoathookConstants {
    public static final int EXTENDER_MOTOR_ID = 13;
    public static final int ROTATION_MOTOR_ID = 14;
    public static final int ROTATION_ENCODER_ID = 15;
    public static final int EXTENDER_ENCODER_ID = 16;
    public static final int BOATHOOK_CANDI_ID = 18;
    public static final double ROTATOR_GEAR_RATIO = 25;
    public static final double ROTATOR_FORWARD_LIMIT = 0.4;
    public static final double ROTATOR_REVERSE_LIMIT = 0.0;
    //    public static final double EXTENDER_CANCODER_RATIO = 101.6 / 40;
    //    public static final double EXTENDER_GEAR_RATIO = 25;
    // 4 inch mechanism drum
    // 40 mm sensor
    public static final double EXTENDER_FORWARD_LIMIT = 4.5; // not in inches (rotations)
    public static final double EXTENDER_REVERSE_LIMIT = -0.15;

    public static final double STAB_ANGLE = 12;
    public static final double IDLE_ANGLE = 90;
    public static final double L2_SETUP_ANGLE = 0.35 * 360;
    public static final double L3_SETUP_ANGLE = 0.33 * 360;
    public static final double L4_SETUP_ANGLE = 0.3 * 360;
    public static final double L2_SCORE_ANGLE = 0.36 * 360;
    public static final double L3_SCORE_ANGLE = 0.33 * 360;
    public static final double L4_SCORE_ANGLE = 0.3 * 360;
    public static final double STAB_EXTENSION = 0.19;
    public static final double IDLE_EXTENSION = -0.15;
    public static final double L2_EXTENSION = 0.58;
    public static final double L3_EXTENSION = 1.85;
    public static final double L4_EXTENSION = 4;
  }
}
