package frc.robot.game;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

public enum ReefBranch {
  /*Right and left refers to the operator's point of view
  * Faces facing the operator will have their offsets flipped 
  * as the robot pose is transformed relative to the april tag, 
  * not the operator POV*/

  TWO_LEFT(9, 22, ReefBranchOffset.RIGHT, true),
  TWO_RIGHT(9, 22, ReefBranchOffset.LEFT, true),
  TWO_L1(9, 22, ReefBranchOffset.L1,false),
  FOUR_LEFT(8, 17, ReefBranchOffset.LEFT, true),
  FOUR_RIGHT(8, 17, ReefBranchOffset.RIGHT, true),
  FOUR_L1(8, 17, ReefBranchOffset.L1,false),
  SIX_LEFT(7, 18, ReefBranchOffset.LEFT, true),
  SIX_RIGHT(7, 18, ReefBranchOffset.RIGHT, true),
  SIX_L1(7, 18, ReefBranchOffset.L1,false),
  EIGHT_LEFT(6, 19, ReefBranchOffset.LEFT, true),
  EIGHT_RIGHT(6, 19, ReefBranchOffset.RIGHT, true),
  EIGHT_L1(6, 19, ReefBranchOffset.L1,false),
  TEN_LEFT(11, 20, ReefBranchOffset.RIGHT, true),
  TEN_RIGHT(11, 20, ReefBranchOffset.LEFT, true),
  TEN_L1(11, 20, ReefBranchOffset.L1,false),
  TWELVE_LEFT(10, 21, ReefBranchOffset.RIGHT, true),
  TWELVE_RIGHT(10, 21, ReefBranchOffset.LEFT, true),
  TWELVE_L1(10, 21, ReefBranchOffset.L1,false),
  LOLLIPOP_DOWN_SETUP(7,18,ReefBranchOffset.PRESET_DOWN_SETUP,false),
  LOLLIPOP_CENTER_SETUP(7,18,ReefBranchOffset.PRESET_CENTER_SETUP,false),
  LOLLIPOP_UP_SETUP(7,18,ReefBranchOffset.PRESET_UP_SETUP,false),
  LOLLIPOP_DOWN(7,18,ReefBranchOffset.PRESET_DOWN_INTAKE,false),
  LOLLIPOP_CENTER(7,18,ReefBranchOffset.PRESET_CENTER_INTAKE,false),
  LOLLIPOP_UP(7,18,ReefBranchOffset.PRESET_UP_INTAKE,false);

  public final int redTag;
  public final int blueTag;
  public final ReefBranchOffset offset;
  public final Pose2d redScoringPose;
  public final Pose2d blueScoringPose;
  public final boolean orbitEnabled;
  private AprilTagFieldLayout fieldLayout;

  ReefBranch(int redTag, int blueTag, ReefBranchOffset offset, boolean orbitEnabled) {
    this.redTag = redTag;
    this.blueTag = blueTag;
    this.offset = offset;
    this.orbitEnabled = orbitEnabled;
    this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    this.redScoringPose = generateTargetScoringPose(redTag, offset);
    this.blueScoringPose = generateTargetScoringPose(blueTag, offset);
  }

  private Pose2d generateTargetScoringPose(int targetTag, ReefBranchOffset targetOffset){
    return this.fieldLayout.getTagPose(targetTag).get().toPose2d().transformBy(targetOffset.offset);
  }
}