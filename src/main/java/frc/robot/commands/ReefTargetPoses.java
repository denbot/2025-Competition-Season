package frc.robot.commands;

import frc.robot.game.ReefAprilTag;

public enum ReefTargetPoses {
  TWO_LEFT(5.0, 2.8, 120, ReefAprilTag.TWO, Direction.LEFT),
  FOUR_LEFT(3.7, 3, 60, ReefAprilTag.FOUR, Direction.LEFT),
  SIX_LEFT(3.2, 4.2, 0, ReefAprilTag.SIX, Direction.LEFT),
  EIGHT_LEFT(3.7, 5.0, -60, ReefAprilTag.EIGHT, Direction.LEFT),
  TEN_LEFT(5.0, 5.2, -120, ReefAprilTag.TEN, Direction.LEFT),
  TWELVE_LEFT(5.7, 3.8, 180, ReefAprilTag.TWELVE, Direction.LEFT),
  TWO_RIGHT(5.2, 3.0, 120, ReefAprilTag.TWO, Direction.RIGHT),
  FOUR_RIGHT(3.9, 2.8, 60, ReefAprilTag.FOUR, Direction.RIGHT),
  SIX_RIGHT(3.2, 3.8, 0, ReefAprilTag.SIX, Direction.RIGHT),
  EIGHT_RIGHT(4.0, 5.2, -60, ReefAprilTag.EIGHT, Direction.RIGHT),
  TEN_RIGHT(5.2, 5.0, -120, ReefAprilTag.TEN, Direction.RIGHT),
  TWELVE_RIGHT(5.7, 4.1, 180, ReefAprilTag.TWELVE, Direction.RIGHT);

  // OTF variables
  public double x;
  public double y;
  public double angle;

  // April Tag Variables
  public final ReefAprilTag aprilTag;
  public final Direction direction;

  ReefTargetPoses(double x, double y, double angle, ReefAprilTag aprilTag, Direction direction) {
    this.x = x;
    this.y = y;
    this.angle = angle;
    this.aprilTag = aprilTag;
    this.direction = direction;
  }

  public enum Direction {
    LEFT,
    RIGHT
  }
}
