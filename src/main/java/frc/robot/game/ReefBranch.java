package frc.robot.game;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

public enum ReefBranch {
  TWO_LEFT(9, 22, ReefBranchOffset.LEFT),
  TWO_RIGHT(9, 22, ReefBranchOffset.RIGHT),
  FOUR_LEFT(8, 17, ReefBranchOffset.LEFT),
  FOUR_RIGHT(8, 17, ReefBranchOffset.RIGHT),
  SIX_LEFT(7, 18, ReefBranchOffset.LEFT),
  SIX_RIGHT(7, 18, ReefBranchOffset.RIGHT),
  EIGHT_LEFT(6, 19, ReefBranchOffset.LEFT),
  EIGHT_RIGHT(6, 19, ReefBranchOffset.RIGHT),
  TEN_LEFT(11, 20, ReefBranchOffset.LEFT),
  TEN_RIGHT(11, 20, ReefBranchOffset.RIGHT),
  TWELVE_LEFT(10, 21, ReefBranchOffset.LEFT),
  TWELVE_RIGHT(10, 21, ReefBranchOffset.RIGHT),
  LOLLIPOP_DOWN_SETUP(7,18,ReefBranchOffset.PRESET_DOWN_SETUP),
  LOLLIPOP_CENTER_SETUP(7,18,ReefBranchOffset.PRESET_CENTER_SETUP),
  LOLLIPOP_UP_SETUP(7,18,ReefBranchOffset.PRESET_UP_SETUP),
  LOLLIPOP_DOWN(7,18,ReefBranchOffset.PRESET_DOWN_INTAKE),
  LOLLIPOP_CENTER(7,18,ReefBranchOffset.PRESET_CENTER_INTAKE),
  LOLLIPOP_UP(7,18,ReefBranchOffset.PRESET_UP_INTAKE);

  public final int redTag;
  public final int blueTag;
  public final ReefBranchOffset offset;
  public final Pose2d redPose;
  public final Pose2d bluePose;
  private AprilTagFieldLayout fieldLayout;

  ReefBranch(int redTag, int blueTag, ReefBranchOffset offset) {
    this.redTag = redTag;
    this.blueTag = blueTag;
    this.offset = offset;
    this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    this.redPose = generateTargetPose(redTag, offset);
    this.bluePose = generateTargetPose(blueTag, offset);
  }

  private Pose2d generateTargetPose(int targetTag, ReefBranchOffset offset){
    return this.fieldLayout.getTagPose(targetTag).get().toPose2d();
  }
}