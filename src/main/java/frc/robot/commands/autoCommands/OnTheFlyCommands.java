package frc.robot.commands.autoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class OnTheFlyCommands {

  private final IntakeCommands intakeCommands;
  private final BoathookCommands boathookCommands;
  private final Drive drive;

  private enum OnTheFlyTargetPose {
    // all defined as x/y locations on the field
    // the relative (0, 0) is the right corner of the blue driver station
    TWO_LEFT(5.05, 2.72, 120),
    FOUR_LEFT(3.64, 2.89, 60),
    SIX_LEFT(3.08, 4.20, 0),
    EIGHT_LEFT(3.93, 5.33, -60),
    TEN_LEFT(5.06, 5.32, -120),
    TWELVE_LEFT(5.9, 3.86, 180),
    TWO_RIGHT(5.33, 2.88, 120),
    FOUR_RIGHT(3.92, 2.73, 60),
    SIX_RIGHT(3.08, 3.87, 0),
    EIGHT_RIGHT(3.65, 5.17, -60),
    TEN_RIGHT(5.34, 5.16, -120),
    TWELVE_RIGHT(5.9, 4.18, 180),
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
    LOLLIPOP_RIGHT_SETUP(2.5, 5.8, 0),
    LOLLIPOP_CENTER_SETUP(2.5, 4.0, 0),
    LOLLIPOP_LEFT_SETUP(2.5, 2.2, 0),
    LOLLIPOP_RIGHT(1.227, 5.8, 0),
    LOLLIPOP_CENTER(1.227, 4.0, 0),
    LOLLIPOP_LEFT(1.227, 2.2, 0);

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

  private double currentRobotX;
  private double currentRobotY;
  private double currentRobotAngle;
  private double offsetX;
  private double offsetY;
  private double offsetAngle;

  private final double translationalKP = 5;
  private final double angularKP = 0.5;

  public OnTheFlyCommands(
      IntakeCommands intakeCommands,
      BoathookCommands boathookCommands,
      Drive drive
  ) {
    this.intakeCommands = intakeCommands;
    this.boathookCommands = boathookCommands;
    this.drive = drive;
  }

  public Command alignTwoLeft() {
    return getAutoAlignCommand(OnTheFlyTargetPose.TWO_LEFT).withName("Align Two Left");
  }

  public Command alignTwoRight() {
    return getAutoAlignCommand(OnTheFlyTargetPose.TWO_RIGHT).withName("Align Two Right");
  }

  public Command alignFourLeft() {
    return getAutoAlignCommand(OnTheFlyTargetPose.FOUR_LEFT).withName("Align Four Left");
  }

  public Command alignFourRight() {
    return getAutoAlignCommand(OnTheFlyTargetPose.FOUR_RIGHT).withName("Align Four Right");
  }

  public Command alignSixLeft() {
    return getAutoAlignCommand(OnTheFlyTargetPose.SIX_LEFT).withName("Align Six Left");
  }

  public Command alignSixRight() {
    return getAutoAlignCommand(OnTheFlyTargetPose.SIX_RIGHT).withName("Align Six Right");
  }

  public Command alignEightLeft() {
    return getAutoAlignCommand(OnTheFlyTargetPose.EIGHT_LEFT).withName("Align Eight Left");
  }

  public Command alignEightRight() {
    return getAutoAlignCommand(OnTheFlyTargetPose.EIGHT_RIGHT).withName("Align Eight Right");
  }

  public Command alignTenLeft() {
    return getAutoAlignCommand(OnTheFlyTargetPose.TEN_LEFT).withName("Align Ten Left");
  }

  public Command alignTenRight() {
    return getAutoAlignCommand(OnTheFlyTargetPose.TEN_RIGHT).withName("Align Ten Right");
  }

  public Command alignTwelveLeft() {
    return getAutoAlignCommand(OnTheFlyTargetPose.TWELVE_LEFT).withName("Align Twelve Left");
  }

  public Command alignTwelveRight() {
    return getAutoAlignCommand(OnTheFlyTargetPose.TWELVE_RIGHT).withName("Align Twelve Right");
  }

  public Command pickupLollipopLeft() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            getAutoAlignCommand(OnTheFlyTargetPose.LOLLIPOP_LEFT_SETUP),
            intakeCommands.intakeDownCommand(),
            boathookCommands.setBoathookStab()
        ),
        new ParallelCommandGroup(
            intakeCommands.runIntakeCommand().withTimeout(2),
            getAutoAlignCommand(OnTheFlyTargetPose.LOLLIPOP_LEFT)
        ),
        boathookCommands.handoffCommand(intakeCommands));
  }

  public Command pickupLollipopRight() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            getAutoAlignCommand(OnTheFlyTargetPose.LOLLIPOP_RIGHT_SETUP),
            intakeCommands.intakeDownCommand(),
            boathookCommands.setBoathookStab()),
        new ParallelCommandGroup(
            intakeCommands.runIntakeCommand().withTimeout(2),
            getAutoAlignCommand(OnTheFlyTargetPose.LOLLIPOP_RIGHT)),
        boathookCommands.handoffCommand(intakeCommands));
  }

  public Command pickupLollipopCenter() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            getAutoAlignCommand(OnTheFlyTargetPose.LOLLIPOP_CENTER_SETUP),
            intakeCommands.intakeDownCommand(),
            boathookCommands.setBoathookStab()),
        new ParallelCommandGroup(
            intakeCommands.runIntakeCommand().withTimeout(2),
            getAutoAlignCommand(OnTheFlyTargetPose.LOLLIPOP_CENTER)),
        boathookCommands.handoffCommand(intakeCommands));
  }

  private Command getFinalAlignmentCommand(OnTheFlyTargetPose targetPose) {
    return Commands.run(
            () -> {
              double x = targetPose.x;
              double y = targetPose.y;
              double angle = targetPose.angle;
              if (DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                // approximate location of top right corner of the reef = 17.6, 7.6
                x = 17.548 - x;
                y = 8.052 - y;
                angle += 180;
                if (angle > 180) angle -= 360;
              }
              currentRobotX = drive.getPose().getX();
              currentRobotY = drive.getPose().getY();
              currentRobotAngle = drive.getPose().getRotation().getDegrees();
              offsetX = x - currentRobotX;
              offsetY = y - currentRobotY;
              offsetAngle = angle - currentRobotAngle;
              offsetAngle =
                  offsetAngle > 180
                      ? offsetAngle - 360
                      : offsetAngle < -180 ? offsetAngle + 360 : offsetAngle;
              System.out.println("X: " + offsetX + "\nY: " + offsetY + "\nAngle: " + offsetAngle);
              ChassisSpeeds newSpeeds =
                  new ChassisSpeeds(
                      offsetX * translationalKP,
                      offsetY * translationalKP,
                      offsetAngle * angularKP);
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      newSpeeds, drive.getRotation()));
            })
        .until(
            () ->
                Math.abs(offsetX) < 0.02 && Math.abs(offsetY) < 0.02 && Math.abs(offsetAngle) < 1);
  }

  private Command getAutoAlignCommand(OnTheFlyTargetPose targetPose) {
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
            new PathConstraints(4.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720))
        )
        .andThen(getFinalAlignmentCommand(targetPose));
  }
}
