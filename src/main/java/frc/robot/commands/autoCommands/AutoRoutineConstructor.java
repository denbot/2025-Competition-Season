package frc.robot.commands.autoCommands;

import frc.robot.commands.visionCommands.TargetChange;
import java.util.ArrayList;

public class AutoRoutineConstructor {

  private static ArrayList<TargetChange> targets = new ArrayList<>();
  private static TargetChange possibleTarget;

  /**
   * addTarget should only be used to directly add a target to the array addPossibleTarget and
   * confirmTarget should be used for building block autos
   */
  public static void addTarget(TargetChange target) {
    targets.add(target);
  }

  public static void addPossibleTarget(TargetChange target) {
    possibleTarget = target;
  }

  public static void confirmTarget() {
    addTarget(possibleTarget);
    System.out.println("Added Target: " + possibleTarget);
  }

  public static AutoCommandScheduler getAutoRoutine() {
    TargetChange[] returnTargets = new TargetChange[targets.size()];
    targets.toArray(returnTargets);
    return new AutoCommandScheduler(returnTargets);
  }
}
