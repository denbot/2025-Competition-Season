package frc.robot.commands.rumbleCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Optional;

@SuppressWarnings("OptionalUsedAsFieldOrParameterType")
public abstract class StateBasedRumbleCommand<E extends Enum<E>> extends Command {
  private E currentState;
  private Optional<E> endState;
  private Optional<RumbleCommand> currentCommand;

  private boolean weShouldEnd = false;

  public StateBasedRumbleCommand() {
    /*
    We explicitly do not add the rumble subsystem requirement. Instead, we listen to the interrupted and completed
    commands. If our sub-command is interrupted, we end this command. If the command finishes, we call the next state.
     */
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command -> {
              if (currentCommand.isEmpty() || currentCommand.get() != command) {
                return;
              }

              weShouldEnd = true; // Our sub-command was interrupted
            });

    CommandScheduler.getInstance()
        .onCommandFinish(
            command -> {
              if (currentCommand.isEmpty() || currentCommand.get() != command) {
                return;
              }

              currentState = getNextState(currentState);
              if (currentState == null) {
                throw new RuntimeException("Returned state was null from getNextState");
              }

              currentCommand = getRumbleCommand(currentState);
            });
  }

  @Override
  public void initialize() {
    currentState = initialState();
    endState = endState();
    currentCommand = getRumbleCommand(currentState);
  }

  @Override
  public void execute() {
    if (currentCommand.isEmpty()) {
      currentState = getNextState(currentState);
      return;
    }

    RumbleCommand command = currentCommand.get();

    if (!command.isScheduled()) {
      command.schedule();
    }
  }

  @Override
  public boolean isFinished() {
    if (endState.isPresent() && endState.get() == currentState) {
      return true;
    }

    return weShouldEnd;
  }

  @Override
  public void end(boolean interrupted) {
    // We can't trust interrupted as the scheduler wouldn't interrupt us, but weShouldEnd only
    // happens if a sub-command is interrupted.
    interrupted = weShouldEnd;

    // If we were interrupted, this command was already canceled
    if (!interrupted && currentCommand.isPresent()) {
      currentCommand.get().cancel();
    }
  }

  protected abstract E initialState();

  protected abstract Optional<E> endState();

  protected abstract E getNextState(E currentState);

  protected abstract Optional<RumbleCommand> getRumbleCommand(E currentState);
}
