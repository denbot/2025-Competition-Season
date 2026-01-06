package frc.robot.game;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum ReefBranchOffset {
  /*  Offsets from April tags for Left Branch, Right Branch, L1 between branches,
      Setup locations and intaking locations for lollipops
  */
  LEFT(new Pose2d(1.5,0.5,new Rotation2d(180.0))),
  RIGHT(new Pose2d(-1.5,0.5,new Rotation2d(180.0))),
  L1((new Pose2d(0,0.5,new Rotation2d(0.0)))),
  PRESET_UP_SETUP((new Pose2d(0,0.5,new Rotation2d(0.0)))),
  PRESET_CENTER_SETUP((new Pose2d(0,0.5,new Rotation2d(0.0)))),
  PRESET_DOWN_SETUP((new Pose2d(0,0.5,new Rotation2d(0.0)))),
  PRESET_UP_INTAKE((new Pose2d(0,0.5,new Rotation2d(0.0)))),
  PRESET_CENTER_INTAKE((new Pose2d(0,0.5,new Rotation2d(0.0)))),
  PRESET_DOWN_INTAKE((new Pose2d(0,0.5,new Rotation2d(0.0))));

  public final Pose2d offset;

  ReefBranchOffset(Pose2d offset) {
    this.offset = offset;
  }
}
