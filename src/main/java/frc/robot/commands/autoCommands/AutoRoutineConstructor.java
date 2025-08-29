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
    if (lastAddedWasTarget) scores.add(placeholderScore);
    targets.add(target);
    lastAddedWasTarget = true;
    shouldClearCommands = false;
  }

  public static void addPossibleTarget(TargetChange target) {
    possibleTarget = target;
    // if you have two auto aligns after the other, ensure the scheduler can follow the alternating
    // order
    System.out.println("Added Placeholder Score");
    shouldClearCommands = false;
    lastButtonWasTarget = true;
  }

  public static void addScore(AutoScoreCommand score) {
    if (!lastAddedWasTarget) targets.add(placeholderTarget);
    scores.add(score);
    lastAddedWasTarget = false;
    shouldClearCommands = false;
  }

  public static void addPossibleScore(AutoScoreCommand score) {
    possibleScore = score;
    // if you have two score commandes after the other, ensure the scheduler can follow the
    // alternating order
    shouldClearCommands = false;
    lastButtonWasTarget = false;
  }

  public static void confirmCommand() {
    if (lastButtonWasTarget) addTarget(possibleTarget);
    else addScore(possibleScore);
    if (shouldClearCommands) targets.clear();
    shouldClearCommands = false;
  }

  public static void clearTargets() {
    shouldClearCommands = true;
  }

  public static AutoCommandScheduler getAutoRoutine() {
    TargetChange[] returnTargets = new TargetChange[targets.size()];
    AutoScoreCommand[] returnScores = new AutoScoreCommand[scores.size()];
    targets.toArray(returnTargets);
    scores.toArray(returnScores);
    return new AutoCommandScheduler(returnTargets, returnScores);
  }
}
