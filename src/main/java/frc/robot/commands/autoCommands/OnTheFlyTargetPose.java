package frc.robot.commands.autoCommands;

import frc.robot.game.ReefAprilTag;

public enum OnTheFlyTargetPose {
  // all defined as x/y locations on the field
  // the relative (0, 0) is the right corner of the blue driver station
  TWO_LEFT(5.1, 2.82, 120, ReefAprilTag.TWO, Direction.LEFT),
  FOUR_LEFT(3.76, 2.98, 60, ReefAprilTag.FOUR, Direction.LEFT),
  SIX_LEFT(3.2, 4.2, 0, ReefAprilTag.SIX, Direction.LEFT),
  EIGHT_LEFT(4.0, 5.2, -60, ReefAprilTag.EIGHT, Direction.LEFT),
  TEN_LEFT(5.2, 5.0, -120, ReefAprilTag.TEN, Direction.LEFT),
  TWELVE_LEFT(5.84, 3.76, 180, ReefAprilTag.TWELVE, Direction.LEFT),
  TWO_RIGHT(5.22, 3.0, 120, ReefAprilTag.TWO, Direction.RIGHT),
  FOUR_RIGHT(3.96, 2.78, 60, ReefAprilTag.FOUR, Direction.RIGHT),
  SIX_RIGHT(3.2, 3.86, 0, ReefAprilTag.SIX, Direction.RIGHT),
  EIGHT_RIGHT(3.68, 5.01, -60, ReefAprilTag.EIGHT, Direction.RIGHT),
  TEN_RIGHT(5.16, 5.16, -120, ReefAprilTag.TEN, Direction.RIGHT),
  TWELVE_RIGHT(5.84, 4.2, 180, ReefAprilTag.TWELVE, Direction.RIGHT),
  // Human Player Locations
  HUMAN_LEFT(1.3, 1.0, -130),
  HUMAN_RIGHT(1.3, 6.6, 130),
  // Climb positions
  BLUE_CLIMB_ONE(8.8, 7.25, 180),
  BLUE_CLIMB_TWO(8.8, 6.18, 180),
  BLUE_CLIMB_THREE(8.8, 5.1, 180),
  RED_CLIMB_ONE(8.8, 0.8, 180),
  RED_CLIMB_TWO(8.8, 1.9, 180),
  RED_CLIMB_THREE(8.8, 2.95, 180),
  // "Lolipop" or coral with algea on top of it positions
  LOLIPOP_ONE(1.227, 5.8, 180),
  LOLIPOP_TWO(1.227, 4.0, 180),
  LOLIPOP_THREE(1.227, 2.2, 180);

  // OTF variables
  public final double x;
  public final double y;
  public final double angle;

  // April Tag Variables
  public final ReefAprilTag aprilTag;
  public final Direction direction;

  OnTheFlyTargetPose(double x, double y, double angle, ReefAprilTag aprilTag, Direction direction) {
    this.x = x;
    this.y = y;
    this.angle = angle;
    this.aprilTag = aprilTag;
    this.direction = direction;
  }

  // constructor for alignment poses without explicit april tag information
  OnTheFlyTargetPose(double x, double y, double angle) {
    this.x = x;
    this.y = y;
    this.angle = angle;
    this.aprilTag = null;
    this.direction = null;
  }

  public enum Direction {
    LEFT,
    RIGHT
  }
}
