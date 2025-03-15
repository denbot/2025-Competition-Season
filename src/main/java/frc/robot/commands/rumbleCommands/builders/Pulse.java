package frc.robot.commands.rumbleCommands.builders;

import edu.wpi.first.wpilibj.GenericHID;

public record Pulse(double time, double power, GenericHID.RumbleType rumbleType) {

  public enum PulseTime {
    FAST(0.2),
    SUSTAINED(0.5);

    public final double time;

    PulseTime(double time) {
      this.time = time;
    }
  }

  public enum PulseWait {
    NEAR_INSTANT(0.1),
    FAST(0.25),
    DELAY(0.5);

    public final double time;

    PulseWait(double time) {
      this.time = time;
    }
  }

  public enum PulseStrength {
    LOW(0.25),
    MEDIUM(0.5),
    HIGH(1.0);

    public final double strength;

    PulseStrength(double strength) {
      this.strength = strength;
    }
  }
}
