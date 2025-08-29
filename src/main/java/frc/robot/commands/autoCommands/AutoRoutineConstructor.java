package frc.robot.commands.autoCommands;

import frc.robot.commands.visionCommands.TargetChange;
import java.util.ArrayList;

public class AutoRoutineConstructor {

  private static ArrayList<TargetChange> targets = new ArrayList<>();
  private static ArrayList<AutoScoreCommand> scores = new ArrayList<>();
  private static TargetChange possibleTarget;
  private static AutoScoreCommand possibleScore;
  private static boolean shouldClearCommands = false;
  private static boolean lastButtonWasTarget = false;
  private static boolean lastAddedWasTarget = false;
  private static TargetChange placeholderTarget = new TargetChange(null, null);
  private static AutoScoreCommand placeholderScore = new AutoScoreCommand(null);

  /**
   * addTarget should only be used to directly add a target to the array addPossibleTarget and
   * confirmTarget should be used for building block autos
   */
  public static void addTarget(TargetChange target) {
    if (lastAddedWasTarget) scores.add(placeholderScore); // ensure alternating order for AutoCommandScheduler
    targets.add(target);
    lastAddedWasTarget = true;
    shouldClearCommands = false;
  }

  public static void addPossibleTarget(TargetChange target) {
    possibleTarget = target;
    shouldClearCommands = false;
    lastButtonWasTarget = true;
  }

  public static void addScore(AutoScoreCommand score) {
    if (!lastAddedWasTarget) targets.add(placeholderTarget); // ensure alternating order for AutoCommandScheduler
    scores.add(score);
    lastAddedWasTarget = false;
    shouldClearCommands = false;
  }

  public static void addPossibleScore(AutoScoreCommand score) {
    possibleScore = score;
    shouldClearCommands = false;
    lastButtonWasTarget = false;
  }

  public static void confirmCommand() {
    // depending on what was pressed last, either add the last target or the last score position
    if (lastButtonWasTarget) addTarget(possibleTarget);
    else addScore(possibleScore);

    // clear commands if clearTargets was called
    if (shouldClearCommands) targets.clear();
    shouldClearCommands = false; // reset clearTargets
  }

  public static void clearTargets() {
    shouldClearCommands = true;
  }

  public static AutoCommandScheduler getAutoRoutine() {
    // define empty arrays specifically with the type TargetChange and AutoScoreCommand
    // such that they can be input into AutoCommandScheduler, just ArrayList.toArray returns Object[]
    // which can not be input into AutoCommandScheduler
    TargetChange[] returnTargets = new TargetChange[targets.size()];
    AutoScoreCommand[] returnScores = new AutoScoreCommand[scores.size()];
    
    targets.toArray(returnTargets);
    scores.toArray(returnScores);

    return new AutoCommandScheduler(returnTargets, returnScores);
  }
}
