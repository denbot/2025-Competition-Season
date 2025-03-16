package frc.robot.commands.rumbleCommands;

import frc.robot.commands.rumbleCommands.builders.RumbleScale;
import frc.robot.commands.rumbleCommands.builders.RumbleScaleInterpolator;
import frc.robot.subsystems.RumbleSubsystem;
import java.util.Optional;

public class MultiRumbleCommand extends StateBasedRumbleCommand<MultiRumbleCommand.State> {
  private final RumbleCommand[] commands;
  private int currentIndex;

  public MultiRumbleCommand(RumbleSubsystem subsystem, RumbleScale[] rumbleScales) {
    commands = new RumbleCommand[rumbleScales.length + 1];
    double leftRumble = 0;
    double rightRumble = 0;

    for (int i = 0; i < rumbleScales.length; i++) {
      RumbleScale scale = rumbleScales[i];
      RumbleScaleInterpolator leftInterpolator;
      if (scale.leftTarget().isPresent()) {
        leftInterpolator =
            new RumbleScaleInterpolator(
                scale.scaleType(), leftRumble, scale.leftTarget().get(), scale.time());

        leftRumble = scale.leftTarget().get();
      } else {
        leftInterpolator = RumbleScaleInterpolator.instantly(leftRumble, scale.time());
      }

      RumbleScaleInterpolator rightInterpolator;
      if (scale.rightTarget().isPresent()) {
        rightInterpolator =
            new RumbleScaleInterpolator(
                scale.scaleType(), rightRumble, scale.rightTarget().get(), scale.time());

        rightRumble = scale.rightTarget().get();
      } else {
        rightInterpolator = RumbleScaleInterpolator.instantly(rightRumble, scale.time());
      }

      commands[i] = new RumbleCommand(subsystem, leftInterpolator, rightInterpolator);
    }

    commands[rumbleScales.length] =
        new RumbleCommand(
            subsystem, RumbleScaleInterpolator.stop(), RumbleScaleInterpolator.stop());
  }

  @Override
  public void initialize() {
    super.initialize();
    currentIndex = 0;
  }

  @Override
  protected State initialState() {
    return State.RUN_COMMAND;
  }

  @Override
  protected Optional<State> endState() {
    return Optional.of(State.END);
  }

  @Override
  protected State getNextState(State currentState) {
    if (currentState == State.RUN_COMMAND) {
      currentIndex++;

      if (currentIndex == commands.length) {
        return State.END;
      }
      return State.RUN_COMMAND;
    }

    throw new RuntimeException("Did not handle all states in MultiRumbleCommand");
  }

  @Override
  protected Optional<RumbleCommand> getRumbleCommand(State currentState) {
    if (currentState == State.RUN_COMMAND) {
      return Optional.of(commands[currentIndex]);
    } else {
      return Optional.empty();
    }
  }

  public enum State {
    RUN_COMMAND,
    END
  }
}
