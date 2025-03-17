package frc.robot.vision;

import frc.robot.LimelightHelpers.RawDetection;

/**
 * Stores the state of a single coral tracker from a specific timestep.
 */
public record SingleCoralTrackingState(
    int classId,
    double tx,
    double ty,
    double vx,
    double vy,
    boolean isTracking,
    RawDetection lastDetection) {

  static final double coralHeightMeters = 0.301625;
  static final double coralWidthMeters = 0.1143;
  static final double cameraFocalLength = 747.479;

  public double getAspectRatio() {
    if (lastDetection == null) {
      return Double.NaN;
    }

    return Math.abs(lastDetection.corner0_X - lastDetection.corner1_X)
        / Math.abs(lastDetection.corner1_Y - lastDetection.corner2_Y);
  }

  /**
   * Determines if the detected piece is upright or on its side by looking at the aspect ratio
   *
   * @return True if upright, false otherwise.
   */
  public boolean isUpright() {
    double aspectRatio = getAspectRatio();
    if (Double.isNaN(aspectRatio)) {
      return false;
    }

    return aspectRatio < 0.75;
  }

  /**
   * Gets an estimated distance to the piece in meters. This uses the aspect ratio and the known
   * size of the piece to determine the expected size of the coral and the triangle similarity to
   * calculate the distance The width and height of the detection are used separatly to calculate a
   * distance and averaged.
   *
   * @return The estimated distance in meters. NaN if there is no valid piece.
   */
  public double getDistance() {
    if (lastDetection == null || !isTracking) {
      return Double.NaN;
    }

    // Coral is at the edge of the frame so we can't calculate the distance like this.
    if (lastDetection.corner0_X <= 10
        || lastDetection.corner1_X >= 1270
        || lastDetection.corner1_Y <= 10
        || lastDetection.corner2_Y >= 780) {
      return Double.NaN;
    }

    if (isUpright()) {
      double coralYPixels = Math.abs(lastDetection.corner1_Y - lastDetection.corner2_Y);
      double coralXPixels = Math.abs(lastDetection.corner0_X - lastDetection.corner1_X);

      double dist1 = coralHeightMeters * cameraFocalLength / coralYPixels;
      double dist2 = coralWidthMeters * cameraFocalLength / coralXPixels;
      return dist1 + dist2 / 2;
    } else {
      double coralYPixels = Math.abs(lastDetection.corner1_Y - lastDetection.corner2_Y);
      double coralXPixels = Math.abs(lastDetection.corner0_X - lastDetection.corner1_X);

      // Since the coral is not upright Y and width should be used to calculate the distance
      double dist1 = coralWidthMeters * cameraFocalLength / coralYPixels;

      // Calculate the distance is meters of the projection of the coral onto the camera plane.
      // This is determined using the aspect ratio of the coral.
      double expectedCoralWidthMeters = getAspectRatio() * coralWidthMeters;
      double dist2 = expectedCoralWidthMeters * cameraFocalLength / coralXPixels;
      return dist1 + dist2 / 2;
    }
  }
}
