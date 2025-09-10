package frc.robot.commands.autoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class OnTheFlyCommands {

  private enum OnTheFlyTargetPose {
    // all defined as x/y locations on the field
    // the relative (0, 0) is the right corner of the blue driver station
    TWO_LEFT(5.33, 2.88, 120),
    FOUR_LEFT(3.64, 2.89, 60),
    SIX_LEFT(3.08, 4.20, 0),
    EIGHT_LEFT(3.93, 5.33, -60),
    TEN_LEFT(5.06, 5.32, -120),
    TWELVE_LEFT(5.9, 4.18, 180),
    TWO_RIGHT(5.05, 2.72, 120),
    FOUR_RIGHT(3.92, 2.73, 60),
    SIX_RIGHT(3.08, 3.87, 0),
    EIGHT_RIGHT(3.65, 5.17, -60),
    TEN_RIGHT(5.34, 5.16, -120),
    TWELVE_RIGHT(5.9, 3.86, 180),
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

    OnTheFlyTargetPose(double x, double y, double angle) {
      this.x = x;
      this.y = y;
      this.angle = angle;
    }
  }

  public static Command alignTwoLeft() {
    return getAutoAlignCommand(OnTheFlyTargetPose.TWO_LEFT);
  }

  public static Command alignTwoRight() {
    return getAutoAlignCommand(OnTheFlyTargetPose.TWO_RIGHT);
  }

  public static Command alignFourLeft() {
    return getAutoAlignCommand(OnTheFlyTargetPose.FOUR_LEFT);
  }

  public static Command alignFourRight() {
    return getAutoAlignCommand(OnTheFlyTargetPose.FOUR_RIGHT);
  }

  public static Command alignSixLeft() {
    return getAutoAlignCommand(OnTheFlyTargetPose.SIX_LEFT);
  }

  public static Command alignSixRight() {
    return getAutoAlignCommand(OnTheFlyTargetPose.SIX_RIGHT);
  }

  public static Command alignEightLeft() {
    return getAutoAlignCommand(OnTheFlyTargetPose.EIGHT_LEFT);
  }

  public static Command alignEightRight() {
    return getAutoAlignCommand(OnTheFlyTargetPose.EIGHT_RIGHT);
  }

  public static Command alignTenLeft() {
    return getAutoAlignCommand(OnTheFlyTargetPose.TEN_LEFT);
  }

  public static Command alignTenRight() {
    return getAutoAlignCommand(OnTheFlyTargetPose.TEN_RIGHT);
  }

  public static Command alignTwelveLeft() {
    return getAutoAlignCommand(OnTheFlyTargetPose.TWELVE_LEFT);
  }

  public static Command alignTwelveRight() {
    return getAutoAlignCommand(OnTheFlyTargetPose.TWELVE_RIGHT);
  }

  private static Command getAutoAlignCommand(OnTheFlyTargetPose targetPose) {
    double x = targetPose.x;
    double y = targetPose.y;
    double angle = targetPose.angle;

    // BASIC FUNCTION ONLY (TODO) Better red flipping

    // if the alliance is red, flip positions accordingly
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      // approximate location of top right corner of the reef = 17.6, 7.6
      x = 17.548 - x;
      y = 8.052 - y;
      angle += 180;
      if (angle > 180) angle -= 360;
    }

    // initializes new pathFindToPose command which both create a path and has the robot follow said
    // path
    return AutoBuilder.pathfindToPose(
        new Pose2d(x, y, new Rotation2d(Units.degreesToRadians(angle))),
        new PathConstraints(2.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));
  }
}
