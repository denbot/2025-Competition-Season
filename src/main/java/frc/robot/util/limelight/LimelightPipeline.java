package frc.robot.util.limelight;

public enum LimelightPipeline {
  APRIL_TAG(0),
  CORAL_DETECTOR(1);

  public final int pipeline;

  LimelightPipeline(int pipeline) {
    this.pipeline = pipeline;
  }
}
