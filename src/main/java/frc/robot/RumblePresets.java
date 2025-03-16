package frc.robot;

import frc.robot.commands.rumbleCommands.PulseRumbleCommand;
import frc.robot.commands.rumbleCommands.builders.Pulse;
import frc.robot.commands.rumbleCommands.builders.RumbleBuilder;
import frc.robot.subsystems.RumbleSubsystem;

public class RumblePresets {
  public final PulseRumbleCommand coralIntaken;
  public final PulseRumbleCommand coralEjected;

  public RumblePresets(RumbleSubsystem subsystem) {
    RumbleBuilder builder = new RumbleBuilder(subsystem);

    coralIntaken =
        builder
            .left(Pulse.PulseTime.FAST, Pulse.PulseStrength.LOW)
            .right(Pulse.PulseTime.FAST, Pulse.PulseStrength.MEDIUM)
            .build();

    coralEjected =
        builder
            .right(Pulse.PulseTime.FAST, Pulse.PulseStrength.MEDIUM)
            .left(Pulse.PulseTime.FAST, Pulse.PulseStrength.LOW)
            .build();
  }
}
