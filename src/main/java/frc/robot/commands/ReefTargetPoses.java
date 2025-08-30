package frc.robot.commands;

import frc.robot.game.ReefAprilTag;

public enum ReefTargetPoses {
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
  TWELVE_RIGHT(5.84, 4.2, 180, ReefAprilTag.TWELVE, Direction.RIGHT);

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
