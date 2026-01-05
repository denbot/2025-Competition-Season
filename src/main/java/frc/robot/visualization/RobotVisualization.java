package frc.robot.visualization;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.boathook.Boathook;
import frc.robot.subsystems.intake.Intake;

import static edu.wpi.first.units.Units.*;

/**
 * All the visualization numbers come from a 60" wide x 80" tall box placed at the side of the robot. That gives us a
 * 1:20 scale for this mechanism window. The `s` function applies the scale for any distances.
 */
public class RobotVisualization {

  private final Boathook boathook;
  private final Intake intake;

  private final Mechanism2d robot;
  private final MechanismLigament2d intakeAngle;
  private final MechanismLigament2d intakeCoral;

  private final MechanismLigament2d boathookAngle;
  private final MechanismLigament2d boathookExtension;
  private final MechanismLigament2d boathookCoralOffset;
  private final MechanismLigament2d boathookCoral;

  private final MechanismLigament2d bumper1;
  private final MechanismLigament2d bumper2;
  private final MechanismLigament2d bumper3;
  private final MechanismLigament2d bumper4;

  private final Color8Bit lightBlue = new Color8Bit(173, 216, 230);
  private final Color8Bit orange = new Color8Bit(0xFF, 0x6E, 0x00);
  private final Color8Bit white = new Color8Bit(0xFF, 0xFF, 0xFF);
  private final Color8Bit black = new Color8Bit(0x00, 0x00, 0x00);
  private final Color8Bit red = new Color8Bit(0xBB, 0x00, 0x00);
  private final Color8Bit blue = new Color8Bit(0x00, 0x00, 0x77);
  private Color8Bit currentAllianceColor = white;  // Start with an unknown alliance

  private final Distance coralLength = Inches.of(10);

  public RobotVisualization(
      Boathook boathook,
      Intake intake
  ) {
    this.boathook = boathook;
    this.intake = intake;

    robot = new Mechanism2d(s(Inches.of(60)), s(Inches.of(80)), lightBlue);

    robot.getRoot("intake", s(Inches.of(41.75)), s(Inches.of(9.25)))
        .append(
            intakeAngle = new MechanismLigament2d("intake-angle", s(Inches.of(4.25)), 0, 2, orange)
        ).append(
            new MechanismLigament2d("intake-1", s(Inches.of(5.35)), 270, 2, orange)
        ).append(
            intakeCoral = new MechanismLigament2d("intake-coral", s(Inches.of(2.5)), 90, 2, orange)
        );

    // Use a trigger for the coral intaken just so we're not constantly spamming color/length updates
    new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), intake::isCoralIntaken)
        .debounce(0.2)
        .onTrue(Commands.runOnce(this::updateIntakeCoralState));

    robot.getRoot("boathook", s(Inches.of(21)), s(Inches.of(14.57)))
        .append(
            // We use a length of 0 here, so our angle lines up with the 0-degrees reference angle (to the right)
            boathookAngle = new MechanismLigament2d("boathook-angle", 0, 0, 2, orange)
        ).append(
            // Offset because our extension is not around the same axis as our rotation
            new MechanismLigament2d("boathook-1", s(Inches.of(1.35)), 90, 2, orange)
        ).append(
            new MechanismLigament2d("boathook-2", s(Inches.of(8)), 270, 2, orange)
        ).append(
            boathookExtension = new MechanismLigament2d("boathook-extension", s(Inches.of(30)), 0, 2, black)
        ).append(
            // Now we need the coral grabber at the end of the boathooks
            new MechanismLigament2d("boathook-3", s(Inches.of(3)), 0, 2, orange)
        ).append(
            new MechanismLigament2d("boathook-4", s(Inches.of(3)), 41, 2, orange)
        ).append(
            // Now if the boathook could know if it had a coral game piece, this is how we'd display it
            boathookCoralOffset = new MechanismLigament2d("boathook-coral-1", 0, 270, 2, white)
        ).append(
            boathookCoral = new MechanismLigament2d("boathook-coral-2", 0, 180, 2, white)
        );

    robot.getRoot("floor", 0, s(Inches.of(1)))
        .append(
            new MechanismLigament2d("floor-1", s(Inches.of(60)), 0, 1, black)
        );

    robot.getRoot("bumper", s(Inches.of(17)), s(Inches.of(2.46)))
        .append(
            bumper1 = new MechanismLigament2d("bumper-1", s(Inches.of(26)), 0, 3, white)
        ).append(
            bumper2 = new MechanismLigament2d("bumper-2", s(Inches.of(2.5)), 90, 3, white)
        ).append(
            bumper3 = new MechanismLigament2d("bumper-3", s(Inches.of(26)), 90, 3, white)
        ).append(
            bumper4 = new MechanismLigament2d("bumper-4", s(Inches.of(2.5)), 90, 3, white)
        );

    SmartDashboard.putData("RobotMech", robot);
  }

  private static double s(Distance distance) {
    return distance.in(Inches) / 20;
  }

  public void update() {
    intakeAngle.setAngle(intake.getRotatorPosition().in(Degrees));
    boathookAngle.setAngle(boathook.getAngle().in(Degrees));
    boathookExtension.setLength(s(boathook.getLength()));

    if(DriverStation.getAlliance().isEmpty()) {
      if(currentAllianceColor != white) {
        updateAllianceColor(white);
      }
    } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      updateAllianceColor(red);
    } else {
      updateAllianceColor(blue);
    }
  }

  private void updateAllianceColor(Color8Bit color) {
    currentAllianceColor = color;
    bumper1.setColor(color);
    bumper2.setColor(color);
    bumper3.setColor(color);
    bumper4.setColor(color);
  }

  private void updateIntakeCoralState() {
    if (intake.isCoralIntaken()) {
      intakeCoral.setColor(white);
      intakeCoral.setLength(s(coralLength));
    } else {
      intakeCoral.setColor(orange);
      intakeCoral.setLength(s(Inches.of(1.5)));
    }
  }

  /*
   * This method is commented out because we don't actually know if a coral is in the boathook or not, so we can't
   * display any information about it.
   */
//  private void updateBoathookCoralState() {
//    if (boathook.hasCoral()) {
//      boathookCoral.setLength(s(coralLength));
//      boathookCoralOffset.setLength(s(coralLength.div(2)));
//    } else {
//      boathookCoral.setLength(0);
//      boathookCoralOffset.setLength(0);
//    }
//  }
}
