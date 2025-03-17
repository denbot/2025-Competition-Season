package frc.robot.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

/**
 * A thread for running multi-object detection as it is longer the 20ms allowable for the main run
 * loop.
 */
public class MultiCoralTrackingThread extends Thread {
  private final MultiCoralTracker multiCoralTracker;
  private AtomicReference<MultiCoralTrackingState> lastMultiCoralTrackingState =
      new AtomicReference<>();
  private StructArrayPublisher<Translation2d> multiCoralTrackerPoints;

  public MultiCoralTrackingThread(int objectId) {
    multiCoralTracker = new MultiCoralTracker(1);
    // is network table instance thread safe??
    multiCoralTrackerPoints =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("MultiCoralTracker", Translation2d.struct)
            .publish();
  }

  private void swapState(MultiCoralTrackingState newState) {
    MultiCoralTrackingState previousState = lastMultiCoralTrackingState.get();
    boolean successfulSwap = lastMultiCoralTrackingState.compareAndSet(previousState, newState);
    if (!successfulSwap) {
      System.out.println("Failed to update state in MultiObjectTrackingThread");
    }
  }

  public void run() {
    // Just run this repeatedly until the thread is interrupted
    while (!interrupted()) {
      var detections = LimelightHelpers.getRawDetections("limelight-rear");

      multiCoralTracker.addRawDetections(detections);
      multiCoralTracker.update();

      // Do a thread-safe update of the tracking state so other threads can access it.
      swapState(multiCoralTracker.getTrackingState());

      SingleCoralTrackingState activeStates[] = lastMultiCoralTrackingState.get().getActiveTracks();
      SmartDashboard.putNumber("MultiObject_ActiveTracks", activeStates.length);

      // Publish multi track points to display on AdvantageScope
      ArrayList<Translation2d> multiTrackerPoints = new ArrayList<Translation2d>();

      for (int i = 0; i < lastMultiCoralTrackingState.get().states().length; i++) {
        if (lastMultiCoralTrackingState.get().states()[i].isTracking()) {
          multiTrackerPoints.add(
              new Translation2d(
                  lastMultiCoralTrackingState.get().states()[i].tx(),
                  lastMultiCoralTrackingState.get().states()[i].ty()));
        } else {
          multiTrackerPoints.add(new Translation2d(-100, -100));
        }

        // Also publish if this specific multi-object filter is actively tracking to SmartDashboard.
        SmartDashboard.putBoolean(
            "MultiObject" + i + "_Filter_Tracking",
            lastMultiCoralTrackingState.get().states()[i].isTracking());
      }
      multiCoralTrackerPoints.set(multiTrackerPoints.toArray(new Translation2d[0]));
    }

    // Clear out all the dashboard stuff before exiting
    for (int i = 0; i < lastMultiCoralTrackingState.get().states().length; i++) {
      SmartDashboard.putBoolean("MultiObject" + i + "_Filter_Tracking", false);
    }
    multiCoralTrackerPoints.set(new Translation2d[0]);

    // Set the reference to null for if any other threads still try to access it.
    swapState(null);
  }

  public MultiCoralTrackingState getCurrentState() {
    return lastMultiCoralTrackingState.get();
  }
}
