package frc.robot.game;

public enum ReefTarget {
  TWO(9, 22),
  FOUR(8, 17),
  SIX(7, 18),
  EIGHT(6, 19),
  TEN(11, 20),
  TWELVE(10, 21);

  public final double red;
  public final double blue;

  ReefTarget(double red, double blue) {
    this.red = red;
    this.blue = blue;
  }
}
