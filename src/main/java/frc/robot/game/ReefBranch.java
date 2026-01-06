package frc.robot.game;

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
  TWELVE_RIGHT(10, 21, ReefBranchOffset.RIGHT);

  public final int redTag;
  public final int blueTag;
  public final ReefBranchOffset offset;

  ReefBranch(int redTag, int blueTag, ReefBranchOffset offset) {
    this.redTag = redTag;
    this.blueTag = blueTag;
    this.offset = offset;
  }
}

