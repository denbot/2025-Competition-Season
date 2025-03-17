package frc.robot.vision;

import frc.robot.LimelightHelpers.RawDetection;
import java.util.ArrayList;

/**
 * Tracks multiple coral objects at once. This is useful if you expect to have multiple in frame.
 * This will (do its best) handle inconsistant, flickering detections for an object without the 
 * tracks jumping between different objects. If multiple detections are seen it will create a track
 * for each detection. On subsequent frames it will optimally match the detections to the appropriate
 * track based on the distance between the track and the new detections.
 */
public class MultiCoralTracker {
  private int maxTrackers = 5;
  private ArrayList<SingleCoralTracker> trackers = new ArrayList<SingleCoralTracker>();

  private int objectId;
  private SingleCoralTracker bestPersistantTrack = null;

  public MultiCoralTracker(int objectId) {
    this.objectId = objectId;
    for (int i = 0; i < maxTrackers; i++) {
      trackers.add(new SingleCoralTracker(objectId));
    }
  }

  /**
   * Reset all the single coral filters.
   */
  public void reset() {
    for (SingleCoralTracker singleCoralTracker : trackers) {
      singleCoralTracker.reset();
    }
  }

  /**
   * Calculates the "score" to use for matching tracks to detections. Currently this just uses distance.
   * @param tracker The tracker to generate the score for
   * @param detection The detection to generate the score for
   * @return The generated score
   */
  public double calculateScore(SingleCoralTracker tracker, RawDetection detection) {
    return Math.hypot(detection.txnc - tracker.getTX(), detection.tync - tracker.getTY());
  }

  /**
   * Create more trackers if there are not enough for the number of detections seen. This ensures
   * we don't run out of trackers if we have a lot of detections.
   * @param newNumTrackers The target number of trackers to end with.
   */
  public void updateNumTrackers(int newNumTrackers) {
    if (newNumTrackers <= maxTrackers) {
      return;
    }

    for (int i = 0; i < newNumTrackers - maxTrackers; i++) {
      trackers.add(new SingleCoralTracker(objectId));
    }
    maxTrackers = newNumTrackers;
  }

  /**
   * Locates a free (unused) tracker from the list of trackers.
   * @return The first free tracker in the list
   */
  private SingleCoralTracker findFreeTracker() {
    for (SingleCoralTracker singleCoralTracker : trackers) {
      if (!singleCoralTracker.isActive()) {
        return singleCoralTracker;
      }
    }
    return null;
  }

  /**
   * Adds a list of detections to the multi object tracking system. The Hungarian Algorithm is used
   * to best match up the detections to the existing tracks. If there are more detections than
   * tracks then add new tracks. If there are more tracks than detections, then ignore some
   * detections.
   *
   * @param detections The detections to match with tracks.
   */
  public void addRawDetections(RawDetection[] detections) {
    ArrayList<RawDetection> filteredDetections = new ArrayList<RawDetection>();
    ArrayList<Integer> activeTrackerIndicies = new ArrayList<Integer>();

    // Get the active trackers and filtered detections that we need to assign
    for (RawDetection rawDetection : detections) {
      if (rawDetection.classId == this.objectId) {
        filteredDetections.add(rawDetection);
      }
    }

    for (int i = 0; i < trackers.size(); i++) {
      if (trackers.get(i).isActive()) {
        activeTrackerIndicies.add(i);
      }
    }

    if (filteredDetections.size() == 0) {
      return;
    }

    // Make sure we have enough trackers
    updateNumTrackers(filteredDetections.size());

    // There are no active trackers so no need to do more complex assignment
    if (activeTrackerIndicies.size() == 0) {
      // For every detection add it to a different tracker
      for (int i = 0; i < filteredDetections.size(); i++) {
        trackers.get(i).addSingleDetection(filteredDetections.get(i));
      }
    }

    // We need to assign detections to tracker, so populate the matrix for the algorithm
    // We want a square matrix for the assignment
    int matrixSize = Math.max(activeTrackerIndicies.size(), filteredDetections.size());

    double[][] trackerDetectionMatrix = new double[matrixSize][matrixSize];
    for (int trackerId = 0; trackerId < matrixSize; trackerId++) {
      for (int detectionId = 0; detectionId < matrixSize; detectionId++) {
        // If this index in a dummy row or column (to make it square), just assign it a zero
        if (trackerId >= activeTrackerIndicies.size() || detectionId >= filteredDetections.size()) {
          trackerDetectionMatrix[trackerId][detectionId] = 0;
        } else {
          // Otherwise we need to calculate the cost between this tracker and detection.
          trackerDetectionMatrix[trackerId][detectionId] =
              calculateScore(
                  trackers.get(activeTrackerIndicies.get(trackerId)),
                  filteredDetections.get(detectionId));
        }
      }
    }

    // Now we can initialize the assignment algorithm
    HungarianAlgorithm algorithm = new HungarianAlgorithm(trackerDetectionMatrix);

    // 2d array of matches where rows are assignments and columns are the {detection, tracker}
    // assignments
    int[][] optimalAssignments = algorithm.findOptimalAssignment();

    // For all the assignments do the appropriate thing.
    // 1. If the assignment is of a active tracker and a detection, then apply that detection to
    // that tracker
    // 2. If the assignment is of a non-active tracker and a detection, then find the next active
    // tracker
    //   and apply that detection to that tracker, activating it.
    for (int i = 0; i < optimalAssignments.length; i++) {
      int detectionID = optimalAssignments[i][0];
      int trackerID = optimalAssignments[i][1];
      if (detectionID < filteredDetections.size() && trackerID < activeTrackerIndicies.size()) {
        trackers.get(trackerID).addSingleDetection(filteredDetections.get(detectionID));
      } else if (trackerID >= activeTrackerIndicies.size()) {
        SingleCoralTracker newTracker = findFreeTracker();

        // If new track is null or detectionId is invalid, then there is an issue with the code...
        assert (newTracker != null);
        assert (detectionID < filteredDetections.size());

        // Add the detection to the new tracker, activating it.
        newTracker.addSingleDetection((filteredDetections.get(detectionID)));
      }

      // If we have more trackers than detections, we don't need to do anything special as
      // those trackers will just not have an associated detection this round.
    }
  }

  /**
   * Generates the state for the tracker, this can be saved and used later (i.e. if the tracker is run in a thread.)
   * @return The saved state of the tracker
   */
  public MultiCoralTrackingState getTrackingState() {
    SingleCoralTrackingState[] states = new SingleCoralTrackingState[trackers.size()];

    for (int i = 0; i < trackers.size(); i++) {
      states[i] = trackers.get(i).getTrackingState();
    }

    return new MultiCoralTrackingState(states);
  }

  public ArrayList<SingleCoralTracker> getTracks() {
    return trackers;
  }

  /**
   * Update all the single coral trackers that make up the multi-coral tracker.
   */
  public void update() {
    for (int i = 0; i < trackers.size(); i++) {
      trackers.get(i).update();
    }

    // Rest the best persistant track to unassigned if it is now not valid.
    if (bestPersistantTrack != null && !bestPersistantTrack.isTracking()) {
      bestPersistantTrack = null;
    }
  }
}
