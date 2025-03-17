package frc.robot.vision;

import java.util.ArrayList;

/**
 * Stores the state of a multi-coral tracker from a specific timestep.
 */
public record MultiCoralTrackingState(SingleCoralTrackingState[] states) {

  public SingleCoralTrackingState[] getActiveTracks() {
    ArrayList<SingleCoralTrackingState> activeStates = new ArrayList<SingleCoralTrackingState>();
    for (SingleCoralTrackingState singleCoralTrackingState : states) {
      if (singleCoralTrackingState.isTracking()) activeStates.add(singleCoralTrackingState);
    }
    return activeStates.toArray(new SingleCoralTrackingState[0]);
  }
}
