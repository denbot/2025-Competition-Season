package frc.robot.commands.rumble;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.RumbleSubsystem;

public class RumbleBuilder {
  private final RumbleSubsystem rumbleSubsystem;

  public RumbleBuilder(RumbleSubsystem rumbleSubsystem) {
    this.rumbleSubsystem = rumbleSubsystem;
  }

  public CanAddPulse builder() {
    // Start by only allowing a pulse as we can't immediately build or start with a wait.
    return new CanAddPulse() {
      @SuppressWarnings("unchecked")
      @Override
      public MainBuilder pulse(
          Pulse.PulseTime time, Pulse.PulseStrength strength, GenericHID.RumbleType side) {
        return new MainBuilder(new Pulse(time.time, strength.strength, side));
      }
    };
  }

  public interface CanAddPulse {
    default <T extends CanBuild & CanAddPulse & CanAddPause> T pulse(Pulse.PulseTime time) {
      return pulse(time, Pulse.PulseStrength.HIGH);
    }

    default <T extends CanBuild & CanAddPulse & CanAddPause> T pulse(
        Pulse.PulseTime time, Pulse.PulseStrength strength) {
      return pulse(time, strength, GenericHID.RumbleType.kBothRumble);
    }

    <T extends CanBuild & CanAddPulse & CanAddPause> T pulse(
        Pulse.PulseTime time, Pulse.PulseStrength strength, GenericHID.RumbleType side);
  }

  public interface CanAddPause {
    <T extends CanAddPulse> T wait(Pulse.PulseWait delay);
  }

  public interface CanBuild {
    RumbleCommand build();
  }

  // Whenever we see this class, we last had a wait
  private class PulseBuilder implements CanAddPulse {
    final Pulse[] pulses;
    final boolean runsInAutonomous;
    final boolean runsWhenDisabled;

    PulseBuilder() {
      this.pulses = new Pulse[0];
      this.runsInAutonomous = false;
      this.runsWhenDisabled = false;
    }

    PulseBuilder(
        Pulse[] existing, Pulse nextPulse, boolean runsInAutonomous, boolean runsWhenDisabled) {
      this.pulses = new Pulse[existing.length + 1];
      System.arraycopy(existing, 0, pulses, 0, existing.length);
      this.pulses[this.pulses.length - 1] = nextPulse;
      this.runsInAutonomous = runsInAutonomous;
      this.runsWhenDisabled = runsWhenDisabled;
    }

    PulseBuilder(PulseBuilder existing, Pulse nextPulse) {
      this(existing.pulses, nextPulse, existing.runsInAutonomous, existing.runsWhenDisabled);
    }

    @SuppressWarnings("unchecked")
    @Override
    public MainBuilder pulse(
        Pulse.PulseTime time, Pulse.PulseStrength strength, GenericHID.RumbleType side) {
      return new MainBuilder(this, new Pulse(time.time, strength.strength, side));
    }
  }

  // Whenever we see this class, we last had a pulse
  private class MainBuilder extends PulseBuilder implements CanBuild, CanAddPause {
    MainBuilder(Pulse firstPulse) {
      super(new PulseBuilder(), firstPulse);
    }

    MainBuilder(PulseBuilder existing, Pulse nextPulse) {
      super(existing, nextPulse);
    }

    @SuppressWarnings("unchecked")
    @Override
    public PulseBuilder wait(Pulse.PulseWait delay) {
      return new PulseBuilder(this, new Pulse(delay.time, 0, GenericHID.RumbleType.kBothRumble));
    }

    @Override
    public RumbleCommand build() {
      return new RumbleCommand(rumbleSubsystem, pulses, runsInAutonomous, runsWhenDisabled);
    }
  }
}
