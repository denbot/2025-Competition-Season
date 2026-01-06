package frc.robot.game;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public enum ReefBranchOffset {
  /*  Offsets from April tags for Left Branch, Right Branch, L1 between branches,
      Setup locations and intaking locations for lollipops
  */
  LEFT(new Transform2d(0.5,-0.17,new Rotation2d(Degree.of(180)))),
  RIGHT(new Transform2d(0.5,0.17,new Rotation2d(Degree.of(180)))),
  L1((new Transform2d(0.5,0.0,new Rotation2d(0.0)))),
  PRESET_UP_SETUP((new Transform2d(1.1, 1.8,new Rotation2d(Degree.of(180))))),
  PRESET_CENTER_SETUP((new Transform2d(1.1,0,new Rotation2d(Degree.of(180))))),
  PRESET_DOWN_SETUP((new Transform2d(1.1,-1.8,new Rotation2d(Degree.of(180))))),
  PRESET_UP_INTAKE((new Transform2d(2.4,1.8,new Rotation2d(Degree.of(180))))),
  PRESET_CENTER_INTAKE((new Transform2d(2.4,0,new Rotation2d(Degree.of(180))))),
  PRESET_DOWN_INTAKE((new Transform2d(2.4,-1.8,new Rotation2d(Degree.of(180)))));

  public final Transform2d offset;

  ReefBranchOffset(Transform2d offset) {
    this.offset = offset;
  }
}
