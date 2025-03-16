package frc.robot.commands.rumbleCommands;

import frc.robot.commands.rumbleCommands.builders.RumbleScale;
import frc.robot.commands.rumbleCommands.builders.RumbleScaleInterpolator;
import frc.robot.subsystems.RumbleSubsystem;
import java.util.Optional;

/**
 * This is just an example command of what you can do with a state based rumble command. This
 * command ends when it's cancelled by the scheduler.
 */
public class LarsonScannerRumbleCommand
    extends StateBasedRumbleCommand<LarsonScannerRumbleCommand.State> {
  private final RumbleCommand startup;
  private final RumbleCommand leftToRight;
  private final RumbleCommand rightToLeft;

  public LarsonScannerRumbleCommand(RumbleSubsystem subsystem) {
    // subsystem does not get added as a requirement, the rumble commands do.

    final int runTime = 2;

    RumbleScaleInterpolator slowDecreasing =
        new RumbleScaleInterpolator(RumbleScale.ScaleType.LINEAR, 1, 0, runTime);
    RumbleScaleInterpolator slowIncreasing =
        new RumbleScaleInterpolator(RumbleScale.ScaleType.LINEAR, 0, 1, runTime);
    startup = new RumbleCommand(subsystem, slowIncreasing, RumbleScaleInterpolator.stop());
    leftToRight = new RumbleCommand(subsystem, slowDecreasing, slowIncreasing);
    rightToLeft = new RumbleCommand(subsystem, slowIncreasing, slowDecreasing);
  }

  @Override
  protected State initialState() {
    return State.LEFT_TO_RIGHT;
  }

  @Override
  protected Optional<State> endState() {
    return Optional.empty(); // This command ends when canceled
  }

  @Override
  protected State getNextState(State currentState) {
    if (currentState == State.STARTUP) {
      return State.LEFT_TO_RIGHT;
    } else if (currentState == State.LEFT_TO_RIGHT) {
      return State.RIGHT_TO_LEFT;
    } else {
      return State.LEFT_TO_RIGHT;
    }
  }

  @Override
  protected Optional<RumbleCommand> getRumbleCommand(State currentState) {
    if (currentState == State.STARTUP) {
      return Optional.of(startup);
    }
    if (currentState == State.LEFT_TO_RIGHT) {
      return Optional.of(leftToRight);
    }
    if (currentState == State.RIGHT_TO_LEFT) {
      return Optional.of(rightToLeft);
    }

    return Optional.empty();
  }

  public enum State {
    STARTUP,
    LEFT_TO_RIGHT,
    RIGHT_TO_LEFT
  }
}
