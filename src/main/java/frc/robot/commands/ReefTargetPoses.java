package frc.robot.commands;

import frc.robot.game.ReefAprilTag;

public enum ReefTargetPoses {
  TWO_LEFT(6.0, 2.8, 120, ReefAprilTag.TWO, Direction.LEFT),
  FOUR_LEFT(3.75, 2.8, 60, ReefAprilTag.FOUR, Direction.LEFT),
  SIX_LEFT(2.8, 3.5, 0, ReefAprilTag.SIX, Direction.LEFT),
  EIGHT_LEFT(3.75, 4.25, -60, ReefAprilTag.EIGHT, Direction.LEFT),
  TEN_LEFT(5.85, 3.75, -120, ReefAprilTag.TEN, Direction.LEFT),
  TWELVE_LEFT(6.55, 4.25, 180, ReefAprilTag.TWELVE, Direction.LEFT),
  TWO_RIGHT(6.5, 2.1, 120, ReefAprilTag.TWO, Direction.RIGHT),
  FOUR_RIGHT(3.25, -2.5, 60, ReefAprilTag.FOUR, Direction.RIGHT),
  SIX_RIGHT(2.8, 4.5, 0, ReefAprilTag.SIX, Direction.RIGHT),
  EIGHT_RIGHT(3.25, 4.55, -60, ReefAprilTag.EIGHT, Direction.RIGHT),
  TEN_RIGHT(5.55, 4.55, -120, ReefAprilTag.TEN, Direction.RIGHT),
  TWELVE_RIGHT(6.55, 3.75, 180, ReefAprilTag.TWELVE, Direction.RIGHT);

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
