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
    public static final String canivoreSerial = "33ED01F9463847532020204B451D0DFF";
  }

  public static class IntakeConstants {
    public static final double rotatorGearRatio = 25;
    public static final double forwardSoftLimit = 90;
    public static final double reverseSoftLimit = 0;

    public static final int INTAKER_LEFT_MOTOR_ID = 17;
    public static final int INTAKER_RIGHT_MOTOR_ID = 18;

    public static final int ROTATION_LEFT_MOTOR_ID = 19;
    public static final int ROTATION_LEFT_ENCODER_ID = 23;
    public static final int ROTATION_RIGHT_MOTOR_ID = 20;
    public static final int ROTATION_RIGHT_ENCODER_ID = 24;

    public static final int INDEXER_LEFT_MOTOR_ID = 21;
    public static final int INDEXER_RIGHT_MOTOR_ID = 22;

    public static final double intakeDownAngle = 0;
    public static final double intakeUpoAngle = 90;
    public static final double intakeFunnelAngle = 45;
  }
}
