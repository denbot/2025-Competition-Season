package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers.RawDetection;

/**
 * Tracks a single coral piece, filtering the position and handles flickering detections using a
 * Kalman Filter. This will not handle multiple detections in a single frame very well.
 */
public class SingleCoralTracker {
  private int objectId;
  private boolean isActive;
  private KalmanFilter<N4, N1, N2> kalmanFilter;
  private Timer timer;
  private double lastTime = -1;
  private double lastMeasurmentTime = 0.0;
  private RawDetection lastValidDetection = null;
  private int detectionCount = 0;

  public SingleCoralTracker(int objectId) {
    this.objectId = objectId;
    this.isActive = false;
    this.timer = new Timer();
    timer.start();

    // Create the plant for the kalman filter. The filters state is (x,y,velocity_x,veloity_y) of
    // the
    // coral in the 2d camera image plane frame. Not sure the velocity really adds much, but if the
    // camera or
    // coral is moving with flickering detections it will theoretically better predict those
    // flickering frames.
    Matrix<N4, N4> plantA = new Matrix<N4, N4>(Nat.N4(), Nat.N4());
    plantA.set(0, 2, 1.0);
    plantA.set(1, 3, 1.0);

    Matrix<N4, N1> plantB = new Matrix<N4, N1>(Nat.N4(), Nat.N1());

    Matrix<N2, N4> plantC = new Matrix<N2, N4>(Nat.N2(), Nat.N4());
    plantC.set(0, 0, 1.0);
    plantC.set(1, 1, 1.0);

    Matrix<N2, N1> plantD = new Matrix<N2, N1>(Nat.N2(), Nat.N1());

    LinearSystem<N4, N1, N2> objectPlant =
        new LinearSystem<N4, N1, N2>(plantA, plantB, plantC, plantD);

    kalmanFilter =
        new KalmanFilter<N4, N1, N2>(
            Nat.N4(),
            Nat.N2(),
            objectPlant,
            VecBuilder.fill(3, 3, 10, 10), // How accurate we think our model is
            VecBuilder.fill(0.1, 0.1), // How accurate we think our measurments are
            0.020);

    // Set to not active right away.
    lostTracking();
  }

  /**
   * Reinitalize the kalman filter to the default values. This is done whenever we see a new object
   * when no object is currently being tracked. This just sets the state to all zeros with very high
   * uncertainty.
   */
  private void initializeKalmanFilter() {
    Matrix<N4, N4> initalP = new Matrix<N4, N4>(Nat.N4(), Nat.N4());
    initalP.set(0, 0, 10.0);
    initalP.set(1, 1, 10.0);
    initalP.set(2, 2, 5.0);
    initalP.set(3, 3, 5.0);
    kalmanFilter.setP(initalP);
    kalmanFilter.setXhat(VecBuilder.fill(0, 0, 0, 0));
    isActive = true;
  }

  /**
   * Adds a single detection to the tracking filter
   *
   * @param detection The detection to add to the tracking filter
   */
  public void addSingleDetection(RawDetection detection) {
    if (!isActive) {
      initializeKalmanFilter();
    }
    if (detection.classId == this.objectId) {
      kalmanFilter.correct(VecBuilder.fill(0), VecBuilder.fill(detection.txnc, detection.tync));
      lastMeasurmentTime = timer.get();
      lastValidDetection = detection;
      detectionCount++;
    }
  }

  /**
   * This is called whenever tracking is lost to deactivate the Kalman Filter and indicate that no
   * object is being tracked anymore.
   */
  private void lostTracking() {
    isActive = false;
    lastValidDetection = null;
    detectionCount = 0;
  }

  /**
   * Processed a list of detections. The detection that is closest to the current estimated location
   * from the filter will be given to the filter. All others will be ignored. If multiple objects
   * are in the frame then flickering detections can result in the filter prediciton jumping between
   * objects.
   *
   * @param detections All the detections from the camera.
   */
  public void addRawDetections(RawDetection[] detections) {
    if (!isActive) {
      initializeKalmanFilter();
    }

    double filterTX = kalmanFilter.getXhat(0);
    double filterTY = kalmanFilter.getXhat(1);

    RawDetection closestDetection = null;
    for (RawDetection rawDetection : detections) {

      if (rawDetection.classId == this.objectId) {
        if (closestDetection == null) {
          closestDetection = rawDetection;
        } else {
          if (Math.hypot(closestDetection.txnc - filterTX, closestDetection.tync - filterTY)
              > Math.hypot(rawDetection.txnc - filterTX, rawDetection.tync - filterTY)) {
            closestDetection = rawDetection;
          }
        }
      }
    }

    if (closestDetection != null) {
      kalmanFilter.correct(
          VecBuilder.fill(0), VecBuilder.fill(closestDetection.txnc, closestDetection.tync));
      lastMeasurmentTime = timer.get();
      lastValidDetection = closestDetection;
      detectionCount++;
    }
  }

  /**
   * Update the filter. This checks to see if we have timed out and should disable the filter. It
   * also performs the predict step of the Kalman Filter.
   */
  public void update() {
    if (!this.isActive) {
      return;
    }

    double newTime = timer.get();
    if (newTime - lastMeasurmentTime > 1.0
        || Math.abs(kalmanFilter.getXhat(0)) > 40.0
        || Math.abs(kalmanFilter.getXhat(1)) > 27.5) {
      lostTracking();
    } else {
      if (lastTime == -1) {
        lastTime = timer.get();
      } else {
        kalmanFilter.predict(VecBuilder.fill(0), newTime - lastTime);
        lastTime = newTime;
      }
    }
  }

  /** Reset the filter to all zeros with large uncertainty. */
  public void reset() {
    initializeKalmanFilter();
  }

  /**
   * Returns true if the filter is active. Active means it has seen a detection in the last second.
   *
   * @return True if active, False if otherwise.
   */
  public boolean isActive() {
    return isActive;
  }

  /**
   * Returns true if we are tracking a piece. This checks if the filter is active and if we have at
   * seen the piece at least 5 times. This will filter out spurious single frame false positive
   * detections.
   *
   * @return True if we are tracking a piece, false otherwise.
   */
  public boolean isTracking() {
    return isActive && detectionCount > 5;
  }

  /**
   * Gets a record of the current state of the tracker. This can be saved and view later if needed.
   *
   * @return
   */
  public SingleCoralTrackingState getTrackingState() {
    return new SingleCoralTrackingState(
        objectId,
        kalmanFilter.getXhat(0),
        kalmanFilter.getXhat(1),
        kalmanFilter.getXhat(2),
        kalmanFilter.getXhat(3),
        isTracking(),
        lastValidDetection);
  }

  /**
   * Gets the X position of the object relative to the center of the frame in degrees
   *
   * @return Degress off center in the horizontal direction.
   */
  public double getTX() {
    return kalmanFilter.getXhat(0);
  }

  /**
   * Gets the Y position of the object relative to the center of the frame in degrees
   *
   * @return Degress off center in the vertical direction.
   */
  public double getTY() {
    return kalmanFilter.getXhat(1);
  }
}
