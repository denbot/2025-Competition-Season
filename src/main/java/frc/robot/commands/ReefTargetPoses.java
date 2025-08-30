package frc.robot.commands;

public enum ReefTargetPoses {
  TWO_LEFT(6.0, 2.8, 120),
  FOUR_LEFT(3.75, 2.8, 60),
  SIX_LEFT(2.8, 3.5, 0),
  EIGHT_LEFT(3.75, 4.25, -60),
  TEN_LEFT(5.85, 3.75, -120),
  TWELVE_LEFT(6.55, 4.25, 180),
  TWO_RIGHT(6.5, 2.1, 120),
  FOUR_RIGHT(3.25, -2.5, 60),
  SIX_RIGHT(2.8, 4.5, 0),
  EIGHT_RIGHT(3.25, 4.55, -60),
  TEN_RIGHT(5.55, 4.55, -120),
  TWELVE_RIGHT(6.55, 3.75, 180);

  public double x;
  public double y;
  public double angle;

  ReefTargetPoses(double x, double y, double angle) {
    this.x = x;
    this.y = y;
    this.angle = angle;
  }
}
