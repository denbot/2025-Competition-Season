package frc.robot.game;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum ReefBranchOffset {
  LEFT(new Pose2d(1.5,0.5,new Rotation2d(0.0))),
  RIGHT(new Pose2d(-1.5,0.5,new Rotation2d(0.0)));

  public final Pose2d offset;

  ReefBranchOffset(Pose2d offset) {
    this.offset = offset;
  }
}
