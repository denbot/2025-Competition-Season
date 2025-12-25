package frc.robot.util;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

/**
 * This class is loosely based on the WPILib Trigger class. It supports any type and can run a command depending on the
 * state. The default state is null, so it will run a command on the first non-null value as well. If an `onValue`
 * is set up for `null`, then the command will not be scheduled when the event loop runs the first time. It will be run
 * if the event loop changes to a non-null value and then back to a null value.
 *
 * @param <T> The type of supplier you want to listen to change events for
 */
public class MappedTrigger<T> {
  private final Map<T, Command> commandMap;

  /**
   * Creates a new trigger based on the given condition.
   *
   * @param loop          The loop instance that polls this trigger.
   * @param stateSupplier the condition represented by this trigger
   */
  public MappedTrigger(EventLoop loop, Supplier<T> stateSupplier) {
    requireNonNullParam(loop, "loop", "GenericTrigger");
    requireNonNullParam(stateSupplier, "stateSupplier", "GenericTrigger");
    this.commandMap = new HashMap<>();

    loop.bind(
        new Runnable() {
          private T m_previous = null;

          @Override
          public void run() {
            T current = stateSupplier.get();

            boolean different = m_previous == null && current != null;
            different = different || m_previous != null && current == null;
            if (!different && m_previous != null) {
              different = !m_previous.equals(current);
            }

            if (different) {
              Command previousCommand = commandMap.getOrDefault(m_previous, Commands.none());
              if (previousCommand.isScheduled()) {
                previousCommand.cancel();
              }

              Command currentCommand = commandMap.getOrDefault(current, Commands.none());
              currentCommand.schedule();
            }

            m_previous = current;
          }
        }
    );
  }

  /**
   * Creates a new trigger based on the given condition.
   *
   * <p>Polled by the default scheduler button loop.
   *
   * @param stateSupplier the condition represented by this trigger
   */
  public MappedTrigger(Supplier<T> stateSupplier) {
    this(CommandScheduler.getInstance().getDefaultButtonLoop(), stateSupplier);
  }

  public MappedTrigger<T> onValue(T value, Command command) {
    requireNonNullParam(command, "command", "onChange");
    commandMap.put(value, command);
    return this;
  }

  /**
   * Functional interface for the body of a trigger binding.
   */
  @FunctionalInterface
  private interface BindingBody<E> {
    /**
     * Executes the body of the binding.
     *
     * @param previous The previous state of the condition.
     * @param current  The current state of the condition.
     */
    void run(E previous, E current);
  }
}
