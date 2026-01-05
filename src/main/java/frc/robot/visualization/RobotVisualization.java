package frc.robot.visualization;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.boathook.Boathook;
import frc.robot.subsystems.intake.Intake;

import static edu.wpi.first.units.Units.*;

/**
 * All the visualization numbers come from a 60" wide x 80" tall box placed at the side of the robot. That gives us a
 * 1:20 scale for this mechanism window. The `s` function applies the scale for any distances.
 */
public class RobotVisualization implements AutoCloseable {

  private static final Color8Bit lightBlue = new Color8Bit(173, 216, 230);
  private static final Color8Bit orange = new Color8Bit(0xFF, 0x6E, 0x00);
  private static final Color8Bit white = new Color8Bit(0xFF, 0xFF, 0xFF);
  private static final Color8Bit black = new Color8Bit(0x00, 0x00, 0x00);
  private static final Color8Bit red = new Color8Bit(0xBB, 0x00, 0x00);
  private static final Color8Bit blue = new Color8Bit(0x00, 0x00, 0x77);
  private static final Color8Bit grey = new Color8Bit(0x48, 0x48, 0x48);
  private static final Distance coralLength = Inches.of(10);
  private final Boathook boathook;
  private final Intake intake;
  private final IntakeMechanism intakeMechanism;
  private final IntakeMechanism intakeTarget;
  private final BoathookMechanism boathookMechanism;
  private final BoathookMechanism boathookTarget;
  private final BumperMechanism bumper;

  public RobotVisualization(
      Boathook boathook,
      Intake intake
  ) {
    this.boathook = boathook;
    this.intake = intake;

    var robot = new Mechanism2d(s(Inches.of(60)), s(Inches.of(80)), lightBlue);

    var intakeRoot = robot.getRoot("intake", s(Inches.of(41.75)), s(Inches.of(9.25)));
    intakeRoot.append(
        intakeMechanism = new IntakeMechanism(false)
    );
    intakeRoot.append(
        intakeTarget = new IntakeMechanism(true)
    );

    var boathookRoot = robot.getRoot("boathook", s(Inches.of(21)), s(Inches.of(14.57)));
    boathookRoot.append(
        boathookMechanism = new BoathookMechanism(false)
    );
    boathookMechanism.append(
        boathookTarget = new BoathookMechanism(true)
    );

    robot.getRoot("floor", 0, s(Inches.of(1)))
        .append(
            new MechanismLigament2d("floor-1", s(Inches.of(60)), 0, 1, black)
        );

    robot.getRoot("bumper", s(Inches.of(17)), s(Inches.of(2.46)))
        .append(
            bumper = new BumperMechanism()
        );
    SmartDashboard.putData("RobotMech", robot);
  }

  public void update() {
    boathookMechanism.update();
    boathookTarget.update();

    intakeMechanism.update();
    intakeMechanism.update();

    bumper.update();
  }

  private static double s(Distance distance) {
    return distance.in(Inches) / 20;
  }

  @Override
  public void close() {
    intakeMechanism.close();
    intakeTarget.close();
    boathookMechanism.close();
    boathookTarget.close();
    bumper.close();
  }

  static class BumperMechanism extends MechanismLigament2d {
    private final MechanismLigament2d bumper1;
    private final MechanismLigament2d bumper2;
    private final MechanismLigament2d bumper3;
    private final MechanismLigament2d bumper4;
    private Color8Bit currentAllianceColor = white;  // Start with an unknown alliance

    BumperMechanism() {
      super("bumpers", 0, 0);

      this
          .append(
              bumper1 = new MechanismLigament2d("bumper-1", s(Inches.of(26)), 0, 3, white)
          ).append(
              bumper2 = new MechanismLigament2d("bumper-2", s(Inches.of(2.5)), 90, 3, white)
          ).append(
              bumper3 = new MechanismLigament2d("bumper-3", s(Inches.of(26)), 90, 3, white)
          ).append(
              bumper4 = new MechanismLigament2d("bumper-4", s(Inches.of(2.5)), 90, 3, white)
          );
    }

    private void updateAllianceColor(Color8Bit color) {
      currentAllianceColor = color;
      bumper1.setColor(color);
      bumper2.setColor(color);
      bumper3.setColor(color);
      bumper4.setColor(color);
    }

    void update() {
      if (DriverStation.getAlliance().isEmpty()) {
        if (currentAllianceColor != white) {
          updateAllianceColor(white);
        }
      } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        updateAllianceColor(red);
      } else {
        updateAllianceColor(blue);
      }
    }
  }

  class IntakeMechanism extends MechanismLigament2d {

    private final MechanismLigament2d intakeAngle;
    private final MechanismLigament2d intake1;
    private final MechanismLigament2d intakeCoral;
    private final Color8Bit mechanismColor;
    private final boolean target;
    private boolean coralIntaken;
    private boolean visibleNow = false;
    private boolean shouldBeVisible;

    IntakeMechanism(boolean target) {
      this(target, "intake" + (target ? "-target" : ""));
    }

    private IntakeMechanism(boolean target, String prefix) {
      super(prefix, 0, 0);
      this.target = target;

      int lineWidth = target ? 1 : 2;
      shouldBeVisible = !target;

      mechanismColor = target ? grey : orange;

      this
          .append(
              intakeAngle = new MechanismLigament2d(prefix + "-angle", 0, 0, lineWidth, mechanismColor)
          ).append(
              intake1 = new MechanismLigament2d(prefix + "-1", 0, 270, lineWidth, mechanismColor)
          ).append(
              intakeCoral = new MechanismLigament2d(prefix + "-coral", 0, 90, lineWidth, mechanismColor)
          );

      coralIntaken = intake.isCoralIntaken();
    }

    void update() {
      // 5 degrees, to be updated once the intake returns an Angle
      shouldBeVisible = !target || intake.getRotationAngle() - intake.getRotationSetpoint() > 5;

      intakeAngle.setAngle(Revolutions.of(target ? intake.getRotationSetpoint() : intake.getRotationAngle()).in(Degrees));

      boolean coralCurrentlyInIntake = intake.isCoralIntaken();

      // Update the whole visibility state
      if (shouldBeVisible && !visibleNow) {
        visibleNow = true;
        intakeAngle.setLength(s(Inches.of(4.25)));
        intake1.setLength(s(Inches.of(5.35)));
        intakeCoral.setLength(s(Inches.of(2.5)));
        coralIntaken = !coralCurrentlyInIntake;  // Force an update
      } else if (!shouldBeVisible && visibleNow) {
        visibleNow = false;
        intakeAngle.setLength(0);
        intake1.setLength(0);
        intakeCoral.setLength(0);
      }

      // The target view should never show a coral
      if (!target) {
        if (coralCurrentlyInIntake && !coralIntaken) {
          coralIntaken = true;
          intakeCoral.setColor(white);
          intakeCoral.setLength(s(coralLength));
          intakeCoral.setLineWeight(6);
        } else if (!coralCurrentlyInIntake && coralIntaken) {
          coralIntaken = false;
          intakeCoral.setColor(mechanismColor);
          intakeCoral.setLength(s(Inches.of(2.5)));
          intakeCoral.setLineWeight(2);
        }
      }
    }
  }

  class BoathookMechanism extends MechanismLigament2d {
    private final MechanismLigament2d boathookAngle;
    private final MechanismLigament2d boathookExtension;
    private final MechanismLigament2d boathookCoralOffset;
    private final MechanismLigament2d boathookCoral;
    private final MechanismLigament2d boathook1;
    private final MechanismLigament2d boathook2;
    private final MechanismLigament2d boathook3;
    private final MechanismLigament2d boathook4;
    private final boolean target;
    // We don't have a way to know if the boathook has a coral or not
    @SuppressWarnings({"FieldCanBeLocal", "FieldMayBeFinal"})
    private boolean hasCoral = true;
    private boolean visibleNow = false;
    private boolean shouldBeVisible;

    BoathookMechanism(boolean target) {
      this(target, "boathook" + (target ? "-target" : ""));
    }

    BoathookMechanism(boolean target, String prefix) {
      super(prefix, 0, 0);
      this.target = target;

      int lineWidth = target ? 1 : 2;
      shouldBeVisible = !target;

      Color8Bit mechanismColor = target ? grey : orange;

      this
          .append(
              // We use a length of 0 here, so our angle lines up with the 0-degrees reference angle (to the right)
              boathookAngle = new MechanismLigament2d(prefix + "-angle", 0, 0, lineWidth, mechanismColor)
          ).append(
              // Offset because our extension is not around the same axis as our rotation
              boathook1 = new MechanismLigament2d(prefix + "-1", 0, 90, lineWidth, mechanismColor)
          ).append(
              boathook2 = new MechanismLigament2d(prefix + "-2", 0, 270, lineWidth, mechanismColor)
          ).append(
              boathookExtension = new MechanismLigament2d(prefix + "-extension", 0, 0, lineWidth, mechanismColor)
          ).append(
              // Now we need the coral grabber at the end of the boathooks
              boathook3 = new MechanismLigament2d(prefix + "-3", 0, 0, lineWidth, mechanismColor)
          ).append(
              boathook4 = new MechanismLigament2d(prefix + "-4", 0, 41, lineWidth, mechanismColor)
          ).append(
              // Now if the boathook could know if it had a coral game piece, this is how we'd display it
              boathookCoralOffset = new MechanismLigament2d(prefix + "-coral-1", 0, 270, 6, white)
          ).append(
              boathookCoral = new MechanismLigament2d(prefix + "-coral-2", 0, 180, 6, white)
          );
    }

    void update() {
      boolean atAngleSetpoint = boathook.getAngleSetpoint().minus(boathook.getAngle()).abs(Degrees) < 5;
      boolean atLengthSetpoint = boathook.getLengthSetpoint().minus(boathook.getLength()).abs(Inches) < 1;
      shouldBeVisible = !target || !atAngleSetpoint || !atLengthSetpoint;

      boathookAngle.setAngle((target ? boathook.getAngleSetpoint() : boathook.getAngle()).in(Degrees));

      if(shouldBeVisible && !visibleNow) {
        visibleNow = true;
        boathook1.setLength(s(Inches.of(1.35)));
        boathook2.setLength(s(Inches.of(8)));
        boathookExtension.setLength(s(target ? boathook.getLengthSetpoint() : boathook.getLength()));
        boathook3.setLength(s(Inches.of(3)));
        boathook4.setLength(s(Inches.of(3)));
      } else if(!shouldBeVisible && visibleNow) {
        visibleNow = false;
        boathook1.setLength(0);
        boathook2.setLength(0);
        boathookExtension.setLength(0);
        boathook3.setLength(0);
        boathook4.setLength(0);
      }

      // The target view should never show a coral
      if(!target) {
        if (hasCoral && visibleNow) {
          boathookCoral.setLength(s(coralLength));
          boathookCoralOffset.setLength(s(coralLength.div(2)));
        } else {
          boathookCoral.setLength(0);
          boathookCoralOffset.setLength(0);
        }
      }
    }
  }
}
