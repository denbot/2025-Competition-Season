package frc.robot.commands.rumbleCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.subsystems.RumbleSubsystem;

public class RumbleCommand extends Command {
  private final CommandGenericHID controller;
  private final Pulse[] pulses;
  private final Timer timer;
  private final boolean runsInAutonomous;
  private final boolean runsWhenDisabled;
  private int pulseIndex = 0;

  protected RumbleCommand(
      RumbleSubsystem subsystem,
      Pulse[] pulses,
      boolean runsInAutonomous,
      boolean runsWhenDisabled) {
    this.runsWhenDisabled = runsWhenDisabled;
    this.controller = subsystem.controller;
    this.runsInAutonomous = runsInAutonomous;
    this.pulses = pulses;
    this.timer = new Timer();
    addRequirements(subsystem);
  }

  @Override
  public boolean runsWhenDisabled() {
    return runsWhenDisabled;
  }

  @Override
  public void initialize() {
    // If we're not running in autonomous, and we're in autonomous, ignore everything
    if (DriverStation.isAutonomous() && !runsInAutonomous) {
      return;
    }

    timer.reset();

    pulseIndex = -1; // Start with -1 so the first one is zero
    nextPulseIndex();
  }

  @Override
  public void execute() {
    // Getting the pulseIndex directly is safe as our builder pattern ensures we always have 1
    // pulse. The pulseIndex
    // is updated in nextPulseIndex, and isFinished is called before the next time execute is
    // called.
    Pulse currentPulse = pulses[pulseIndex];

    if (timer.hasElapsed(currentPulse.time())) {
      nextPulseIndex();
    }
  }

  private void nextPulseIndex() {
    pulseIndex++;

    if (pulseIndex >= pulses.length) {
      return; // Command will finish in isFinished
    }

    Pulse pulse = pulses[pulseIndex];

    controller.setRumble(pulse.rumbleType(), pulse.power());
    timer.restart();
  }

  @Override
  public boolean isFinished() {
    if (DriverStation.isAutonomous() && !runsInAutonomous) {
      return true; // Just kill the command so we don't have to check each time
    }

    return pulseIndex >= pulses.length; // We ran out of pulses
  }

  @Override
  public void end(boolean interrupted) {
    controller.setRumble(GenericHID.RumbleType.kBothRumble, 0);
  }
}
