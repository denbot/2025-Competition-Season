package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.game.ReefAprilTag;

public enum OnTheFlyTargetPose {
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
  // Human Player Locations, april tag vars are currently placeholder
  HUMAN_LEFT(1.3, 1.0, -130),
  HUMAN_RIGHT(1.3, 6.6, 130),
  // Start positions
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
  public double x;
  public double y;
  public double angle;

  // April Tag Variables
  public ReefAprilTag aprilTag = null;
  public Direction direction = null;

  OnTheFlyTargetPose(double x, double y, double angle, ReefAprilTag aprilTag, Direction direction) {
    this.x = x;
    this.y = y;
    this.angle = angle;
    this.aprilTag = aprilTag;
    this.direction = direction;

    // if the alliance is red, flip positions accordingly
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      // approximate location of top right corner of the reef = 17.6, 7.6
      this.x = 17.6 - x;
      this.y = 8.05 - y;
      this.angle += 180;
      if (this.angle > 180) this.angle -= 360;
    }
  }

  // constructor for alignment poses without explicit april tag information
  OnTheFlyTargetPose(double x, double y, double angle) {
    this.x = x;
    this.y = y;
    this.angle = angle;

    // if the alliance is red, flip positions accordingly
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      // approximate location of top right corner of the reef = 17.6, 7.6
      this.x = 17.6 - x;
      this.y = 8.05 - y;
      this.angle += 180;
      if (this.angle > 180) this.angle -= 360;
    }
  }

  public enum Direction {
    LEFT,
    RIGHT
  }
}