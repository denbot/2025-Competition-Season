package frc.robot.commands.autoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.game.ReefBranchOffset;
import frc.robot.game.ReefBranch;
import frc.robot.subsystems.boathook.Boathook;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;

import java.util.Map;
import java.util.Optional;

public class OnTheFlyCommands {

  private final Intake intake;
  private final Boathook boathook;
  private final Drive drive;
  private static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  private enum OnTheFlyTargetPose {
    TWO_LEFT(
      generateTargetPose(ReefBranch.TWO_LEFT.redTag, ReefBranchOffset.LEFT), 
      generateTargetPose(ReefBranch.TWO_LEFT.blueTag, ReefBranchOffset.LEFT)),
    FOUR_LEFT(
      generateTargetPose(ReefBranch.FOUR_LEFT.redTag, ReefBranchOffset.LEFT), 
      generateTargetPose(ReefBranch.FOUR_LEFT.blueTag, ReefBranchOffset.LEFT)),
    SIX_LEFT(
      generateTargetPose(ReefBranch.SIX_LEFT.redTag, ReefBranchOffset.LEFT), 
      generateTargetPose(ReefBranch.SIX_LEFT.blueTag, ReefBranchOffset.LEFT)),
    EIGHT_LEFT(
      generateTargetPose(ReefBranch.EIGHT_LEFT.redTag, ReefBranchOffset.LEFT), 
      generateTargetPose(ReefBranch.EIGHT_LEFT.blueTag, ReefBranchOffset.LEFT)),
    TEN_LEFT(
      generateTargetPose(ReefBranch.TEN_LEFT.redTag, ReefBranchOffset.LEFT), 
      generateTargetPose(ReefBranch.TEN_LEFT.blueTag, ReefBranchOffset.LEFT)),
    TWELVE_LEFT(
      generateTargetPose(ReefBranch.TWELVE_LEFT.redTag, ReefBranchOffset.LEFT), 
      generateTargetPose(ReefBranch.TWELVE_LEFT.blueTag, ReefBranchOffset.LEFT)),
    TWO_RIGHT(
      generateTargetPose(ReefBranch.TWO_RIGHT.redTag, ReefBranchOffset.RIGHT), 
      generateTargetPose(ReefBranch.TWO_RIGHT.blueTag, ReefBranchOffset.RIGHT)),
    FOUR_RIGHT(
      generateTargetPose(ReefBranch.FOUR_RIGHT.redTag, ReefBranchOffset.RIGHT), 
      generateTargetPose(ReefBranch.FOUR_RIGHT.blueTag, ReefBranchOffset.RIGHT)),
    SIX_RIGHT(
      generateTargetPose(ReefBranch.SIX_RIGHT.redTag, ReefBranchOffset.RIGHT), 
      generateTargetPose(ReefBranch.SIX_RIGHT.blueTag, ReefBranchOffset.RIGHT)),
    EIGHT_RIGHT(
      generateTargetPose(ReefBranch.EIGHT_RIGHT.redTag, ReefBranchOffset.RIGHT), 
      generateTargetPose(ReefBranch.EIGHT_RIGHT.blueTag, ReefBranchOffset.RIGHT)),
    TEN_RIGHT(
      generateTargetPose(ReefBranch.TEN_RIGHT.redTag, ReefBranchOffset.RIGHT), 
      generateTargetPose(ReefBranch.TEN_RIGHT.blueTag, ReefBranchOffset.RIGHT)),
    TWELVE_RIGHT(
      generateTargetPose(ReefBranch.TWELVE_RIGHT.redTag, ReefBranchOffset.RIGHT), 
      generateTargetPose(ReefBranch.TWELVE_RIGHT.blueTag, ReefBranchOffset.RIGHT)),//,

    // // "Lolipop" or coral with algea on top of it positions
    LOLLIPOP_RIGHT_SETUP(
      new Pose2d(2.5, 2.2, new Rotation2d(180)),
      new Pose2d(15, 5.8, new Rotation2d(0))
      ),
    LOLLIPOP_CENTER_SETUP(
      new Pose2d(2.5, 4.0, new Rotation2d(180)),
      new Pose2d(15, 4.0, new Rotation2d(0))
      ),
    LOLLIPOP_LEFT_SETUP(
      new Pose2d(2.5, 5.8, new Rotation2d(180)),
      new Pose2d(15, 2.2, new Rotation2d(0))
      ),
    LOLLIPOP_RIGHT(
      new Pose2d(16.8, 2.2, new Rotation2d(180)),
      new Pose2d(1.227, 5.8, new Rotation2d(0))
      ),
    LOLLIPOP_CENTER(
      new Pose2d(16.8, 4.0, new Rotation2d(180)),
      new Pose2d(1.227, 4.0, new Rotation2d(0))
      ),
    LOLLIPOP_LEFT(
      new Pose2d(16.8, 5.8, new Rotation2d(180)),
      new Pose2d(1.227, 2.2, new Rotation2d(0))
      );

    public Pose2d redTagPose;
    public Pose2d blueTagPose;

    OnTheFlyTargetPose(Pose2d redTagPose, Pose2d blueTagPose) {
      this.redTagPose = redTagPose;
      this.blueTagPose = blueTagPose;
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
      Intake intake,
      Boathook boathook,
      Drive drive
  ) {
    this.intake = intake;
    this.boathook = boathook;
    this.drive = drive;
  }

  /**
   * Returns a map of reef branches to their corresponding auto-alignment commands.
   *
   * @return A map where each {@link ReefBranch} is associated with its specific alignment command.
   */
  public Map<ReefBranch, Command> branchToAlignmentCommands() {
    return Map.ofEntries(
        Map.entry(ReefBranch.TWO_LEFT, alignTwoLeft()),
        Map.entry(ReefBranch.TWO_RIGHT, alignTwoRight()),
        Map.entry(ReefBranch.FOUR_LEFT, alignFourLeft()),
        Map.entry(ReefBranch.FOUR_RIGHT, alignFourRight()),
        Map.entry(ReefBranch.SIX_LEFT, alignSixLeft()),
        Map.entry(ReefBranch.SIX_RIGHT, alignSixRight()),
        Map.entry(ReefBranch.EIGHT_LEFT, alignEightLeft()),
        Map.entry(ReefBranch.EIGHT_RIGHT, alignEightRight()),
        Map.entry(ReefBranch.TEN_LEFT, alignTenLeft()),
        Map.entry(ReefBranch.TEN_RIGHT, alignTenRight()),
        Map.entry(ReefBranch.TWELVE_LEFT, alignTwelveLeft()),
        Map.entry(ReefBranch.TWELVE_RIGHT, alignTwelveRight())
    );
  }

  private static Pose2d generateTargetPose(int targetTag, ReefBranchOffset offset){
    return fieldLayout.getTagPose(targetTag).get().toPose2d();
  }

  private Boolean isRed(){
    var alliance = DriverStation.getAlliance();
    return alliance.equals(Optional.of(DriverStation.Alliance.Red));
  }

  public Command alignTwoLeft() {
    return getAutoAlignCommand(OnTheFlyTargetPose.TWO_LEFT)
      .withName("Align Two Left");
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
            intake.intakeDownCommand(),
            boathook.setBoathookStab()
        ),
        new ParallelCommandGroup(
            intake.runIntakeCommand().withTimeout(2),
            getAutoAlignCommand(OnTheFlyTargetPose.LOLLIPOP_LEFT)
        ),
        boathook.handoffCommand(intake));
  }

  public Command pickupLollipopRight() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            getAutoAlignCommand(OnTheFlyTargetPose.LOLLIPOP_RIGHT_SETUP),
            intake.intakeDownCommand(),
            boathook.setBoathookStab()),
        new ParallelCommandGroup(
            intake.runIntakeCommand().withTimeout(2),
            getAutoAlignCommand(OnTheFlyTargetPose.LOLLIPOP_RIGHT)),
        boathook.handoffCommand(intake));
  }

  public Command pickupLollipopCenter() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            getAutoAlignCommand(OnTheFlyTargetPose.LOLLIPOP_CENTER_SETUP),
            intake.intakeDownCommand(),
            boathook.setBoathookStab()),
        new ParallelCommandGroup(
            intake.runIntakeCommand().withTimeout(2),
            getAutoAlignCommand(OnTheFlyTargetPose.LOLLIPOP_CENTER)),
        boathook.handoffCommand(intake));
  }

  private Command getAutoAlignCommand(OnTheFlyTargetPose targetTag) {
    // initializes new pathFindToPose command which both create a path and has the robot follow said
    // path
    Pose2d targetPose = isRed() ? targetTag.redTagPose : targetTag.blueTagPose;
    return AutoBuilder.pathfindToPose(
            targetPose,
            new PathConstraints(4.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720))
        )
        .andThen(getFinalAlignmentCommand(targetPose));
  }

  private Command getFinalAlignmentCommand(Pose2d targetPose) {
    return Commands.run(
            () -> {
              currentRobotX = drive.getPose().getX();
              currentRobotY = drive.getPose().getY();
              currentRobotAngle = drive.getPose().getRotation().getDegrees();
              offsetX = targetPose.getX() - currentRobotX;
              offsetY = targetPose.getY() - currentRobotY;
              offsetAngle = targetPose.getRotation().getDegrees() - currentRobotAngle;
              offsetAngle =
                  offsetAngle > 180
                      ? offsetAngle - 360
                      : offsetAngle < -180 ? offsetAngle + 360 : offsetAngle;
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
}
