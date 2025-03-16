package frc.robot.commands.rumbleCommands.builders;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.commands.rumbleCommands.MultiRumbleCommand;
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
    return new MainBuilder(pulseBuilder, RumbleScale.instant(side, strength.strength, time.time));
  }

  // Whenever we see this class, we last had a wait
  public static class PulseBuilder implements CanAddPulse {
    final RumbleSubsystem rumbleSubsystem;
    final RumbleScale[] rumbleScales;

    PulseBuilder(RumbleSubsystem rumbleSubsystem) {
      this.rumbleSubsystem = rumbleSubsystem;
      this.rumbleScales = new RumbleScale[0];
    }

    PulseBuilder(PulseBuilder existing, RumbleScale nextRumble) {
      this.rumbleSubsystem = existing.rumbleSubsystem;
      this.rumbleScales = new RumbleScale[existing.rumbleScales.length + 1];
      System.arraycopy(existing.rumbleScales, 0, rumbleScales, 0, existing.rumbleScales.length);
      this.rumbleScales[this.rumbleScales.length - 1] = nextRumble;
    }

    @Override
    public MainBuilder pulse(
        Pulse.PulseTime time, Pulse.PulseStrength strength, GenericHID.RumbleType side) {
      return new MainBuilder(this, RumbleScale.instant(side, strength.strength, time.time));
    }
  }

  // Whenever we see this class, we last had a pulse
  public static class MainBuilder extends PulseBuilder {
    MainBuilder(PulseBuilder existing, RumbleScale nextRumble) {
      super(existing, nextRumble);
    }

    public PulseBuilder wait(Pulse.PulseWait delay) {
      return new PulseBuilder(
          this, RumbleScale.instant(GenericHID.RumbleType.kBothRumble, 0, delay.time));
    }

    public MultiRumbleCommand build() {
      return new MultiRumbleCommand(rumbleSubsystem, rumbleScales);
    }
  }
}
