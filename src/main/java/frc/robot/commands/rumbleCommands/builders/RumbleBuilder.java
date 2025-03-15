package frc.robot.commands.rumbleCommands.builders;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.commands.rumbleCommands.RumbleCommand;
import frc.robot.subsystems.RumbleSubsystem;

public class RumbleBuilder implements CanAddPulse {
  private final RumbleSubsystem rumbleSubsystem;

  public RumbleBuilder(RumbleSubsystem rumbleSubsystem) {
    this.rumbleSubsystem = rumbleSubsystem;
  }

  @Override
  public MainBuilder pulse(
      Pulse.PulseTime time, Pulse.PulseStrength strength, GenericHID.RumbleType side) {
    PulseBuilder pulseBuilder = new PulseBuilder(rumbleSubsystem);
    return new MainBuilder(pulseBuilder, new Pulse(time.time, strength.strength, side));
  }

  // Whenever we see this class, we last had a wait
  public static class PulseBuilder implements CanAddPulse {
    final RumbleSubsystem rumbleSubsystem;
    final Pulse[] pulses;
    final boolean runsInAutonomous;
    final boolean runsWhenDisabled;

    PulseBuilder(RumbleSubsystem rumbleSubsystem) {
      this.rumbleSubsystem = rumbleSubsystem;
      this.pulses = new Pulse[0];
      this.runsInAutonomous = false;
      this.runsWhenDisabled = false;
    }

    PulseBuilder(PulseBuilder existing, Pulse nextPulse) {
      this.rumbleSubsystem = existing.rumbleSubsystem;
      this.pulses = new Pulse[existing.pulses.length + 1];
      System.arraycopy(existing.pulses, 0, pulses, 0, existing.pulses.length);
      this.pulses[this.pulses.length - 1] = nextPulse;
      this.runsInAutonomous = existing.runsInAutonomous;
      this.runsWhenDisabled = existing.runsWhenDisabled;
    }

    @Override
    public MainBuilder pulse(
        Pulse.PulseTime time, Pulse.PulseStrength strength, GenericHID.RumbleType side) {
      return new MainBuilder(this, new Pulse(time.time, strength.strength, side));
    }
  }

  // Whenever we see this class, we last had a pulse
  public static class MainBuilder extends PulseBuilder {
    MainBuilder(PulseBuilder existing, Pulse nextPulse) {
      super(existing, nextPulse);
    }

    public PulseBuilder wait(Pulse.PulseWait delay) {
      return new PulseBuilder(this, new Pulse(delay.time, 0, GenericHID.RumbleType.kBothRumble));
    }

    public RumbleCommand build() {
      return new RumbleCommand(rumbleSubsystem, pulses, runsInAutonomous, runsWhenDisabled);
    }
  }
}
