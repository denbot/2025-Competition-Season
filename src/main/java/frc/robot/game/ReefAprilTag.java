package frc.robot.game;

public enum ReefAprilTag {
  TWO(9, 22, 120),
  FOUR(8, 17, 60),
  SIX(7, 18, 0),
  EIGHT(6, 19, -60),
  TEN(11, 20, -129),
  TWELVE(10, 21, 180);

  public final double red;
  public final double blue;
  public final double angle; // -180 to 180, from the driver station perspective

  ReefAprilTag(double red, double blue, double angle) {
    this.red = red;
    this.blue = blue;
    this.angle = angle;
  }
}
