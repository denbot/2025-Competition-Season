package frc.robot.commands.rumbleCommands;

import edu.wpi.first.wpilibj.GenericHID;

public interface CanAddPulse {
  default RumbleBuilder.MainBuilder pulse(Pulse.PulseTime time) {
    return pulse(time, Pulse.PulseStrength.HIGH);
  }

  default RumbleBuilder.MainBuilder pulse(Pulse.PulseTime time, Pulse.PulseStrength strength) {
    return pulse(time, strength, GenericHID.RumbleType.kBothRumble);
  }

  default RumbleBuilder.MainBuilder left(Pulse.PulseTime time) {
    return left(time, Pulse.PulseStrength.HIGH);
  }

  default RumbleBuilder.MainBuilder left(Pulse.PulseTime time, Pulse.PulseStrength strength) {
    return pulse(time, strength, GenericHID.RumbleType.kLeftRumble);
  }

  default RumbleBuilder.MainBuilder right(Pulse.PulseTime time) {
    return right(time, Pulse.PulseStrength.HIGH);
  }

  default RumbleBuilder.MainBuilder right(Pulse.PulseTime time, Pulse.PulseStrength strength) {
    return pulse(time, strength, GenericHID.RumbleType.kRightRumble);
  }

  RumbleBuilder.MainBuilder pulse(
      Pulse.PulseTime time, Pulse.PulseStrength strength, GenericHID.RumbleType side);
}
