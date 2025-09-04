package frc.robot.commands.autoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public enum ClimbPositions {
  BLUE_ONE(8.8, 7.25, Team.BLUE),
  BLUE_TWO(8.8, 6.18, Team.BLUE),
  BLUE_THREE(8.8, 5.1, Team.BLUE),
  RED_ONE(8.8, 0.8, Team.RED),
  RED_TWO(8.8, 1.9, Team.RED),
  RED_THREE(8.8, 2.95, Team.RED);

  public double x;
  public double y;
  public double angle;
  public Team team;
  public Pose2d pose;

  private ClimbPositions(double x, double y, Team team) {
    this.x = x;
    this.y = y;
    this.team = team;
    if (this.team == Team.BLUE) this.angle = 180;
    else this.angle = 0;
    this.pose = new Pose2d(this.x, this.y, new Rotation2d(Units.degreesToRadians(this.angle)));
  }

  private enum Team {
    RED,
    BLUE
  }
}
