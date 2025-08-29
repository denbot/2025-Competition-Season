package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.boathookCommands.SetLevelCommand;
import frc.robot.commands.visionCommands.TargetChange;
import java.util.ArrayList;

public class AutoRoutineConstructor {

  private static ArrayList<AutoBuildingBlockCommand> commands = new ArrayList<>();
  private static TargetChange possibleTarget;
  private static SetLevelCommand possibleLevel;
  private static boolean newAutoAlign;
  private static boolean newScoreCommand;
  private static boolean shouldClearCommands = false;

  public static void addPossibleTarget(TargetChange possibleTarget) {
    AutoRoutineConstructor.possibleTarget = possibleTarget;
    newAutoAlign = true;
  }

  public static void addPossibleScore(SetLevelCommand possibleScore) {
    possibleLevel = possibleScore;
    newScoreCommand = true;
  }

  public static void clearCurrentCommand() {
    possibleTarget = null;
    possibleLevel = null;
  }

  public static void clearAllCommands() {
    shouldClearCommands = true;
  }

  public static void confirmCommand() {
    // if there is a new possible target, pass that in, if not, pass in null, same for
    // newScoreCommand
    commands.add(
        new AutoBuildingBlockCommand(
            newAutoAlign ? possibleTarget : null, newScoreCommand ? possibleLevel : null));

    newAutoAlign = false;
    newScoreCommand = false;
    if (shouldClearCommands) commands.clear();
    shouldClearCommands = false;
  }

  public static SequentialCommandGroup getAutoRoutine() {
    Command[] returnCommands = new Command[commands.size()];
    commands.toArray(returnCommands);
    return new SequentialCommandGroup(returnCommands);
  }
}
