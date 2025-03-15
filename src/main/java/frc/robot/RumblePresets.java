package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.commands.rumbleCommands.Pulse;
import frc.robot.commands.rumbleCommands.RumbleBuilder;
import frc.robot.commands.rumbleCommands.RumbleCommand;
import frc.robot.commands.rumbleCommands.ScaledRumbleCommand;
import frc.robot.subsystems.RumbleSubsystem;

public class RumblePresets {
  public final RumbleCommand coralIntaken;
  public final RumbleCommand coralEjected;

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
