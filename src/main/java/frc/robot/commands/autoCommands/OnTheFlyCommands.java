package frc.robot.commands.autoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
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

  private Boolean isRed(){
    var alliance = DriverStation.getAlliance();
    return alliance.equals(Optional.of(DriverStation.Alliance.Red));
  }

  public Command alignTwoLeft() {
    return getAutoAlignCommand(ReefBranch.TWO_LEFT)
      .withName("Align Two Left");
  }

  public Command alignTwoRight() {
    return getAutoAlignCommand(ReefBranch.TWO_RIGHT).withName("Align Two Right");
  }

  public Command alignFourLeft() {
    return getAutoAlignCommand(ReefBranch.FOUR_LEFT).withName("Align Four Left");
  }

  public Command alignFourRight() {
    return getAutoAlignCommand(ReefBranch.FOUR_RIGHT).withName("Align Four Right");
  }

  public Command alignSixLeft() {
    return getAutoAlignCommand(ReefBranch.SIX_LEFT).withName("Align Six Left");
  }

  public Command alignSixRight() {
    return getAutoAlignCommand(ReefBranch.SIX_RIGHT).withName("Align Six Right");
  }

  public Command alignEightLeft() {
    return getAutoAlignCommand(ReefBranch.EIGHT_LEFT).withName("Align Eight Left");
  }

  public Command alignEightRight() {
    return getAutoAlignCommand(ReefBranch.EIGHT_RIGHT).withName("Align Eight Right");
  }

  public Command alignTenLeft() {
    return getAutoAlignCommand(ReefBranch.TEN_LEFT).withName("Align Ten Left");
  }

  public Command alignTenRight() {
    return getAutoAlignCommand(ReefBranch.TEN_RIGHT).withName("Align Ten Right");
  }

  public Command alignTwelveLeft() {
    return getAutoAlignCommand(ReefBranch.TWELVE_LEFT).withName("Align Twelve Left");
  }

  public Command alignTwelveRight() {
    return getAutoAlignCommand(ReefBranch.TWELVE_RIGHT).withName("Align Twelve Right");
  }

  public Command pickupLollipopUp() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            getAutoAlignCommand(ReefBranch.LOLLIPOP_UP_SETUP),
            intake.intakeDownCommand(),
            boathook.setBoathookStab()
        ),
        new ParallelCommandGroup(
            intake.runIntakeCommand().withTimeout(2),
            getAutoAlignCommand(ReefBranch.LOLLIPOP_UP)
        ),
        boathook.handoffCommand(intake));
  }

  public Command pickupLollipopDown() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            getAutoAlignCommand(ReefBranch.LOLLIPOP_DOWN_SETUP),
            intake.intakeDownCommand(),
            boathook.setBoathookStab()),
        new ParallelCommandGroup(
            intake.runIntakeCommand().withTimeout(2),
            getAutoAlignCommand(ReefBranch.LOLLIPOP_DOWN)),
        boathook.handoffCommand(intake));
  }

  public Command pickupLollipopCenter() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            getAutoAlignCommand(ReefBranch.LOLLIPOP_CENTER_SETUP),
            intake.intakeDownCommand(),
            boathook.setBoathookStab()),
        new ParallelCommandGroup(
            intake.runIntakeCommand().withTimeout(2),
            getAutoAlignCommand(ReefBranch.LOLLIPOP_CENTER)),
        boathook.handoffCommand(intake));
  }

  public Command getAutoAlignCommand(ReefBranch targetTag) {
    // initializes new pathFindToPose command which both create a path and has the robot follow said
    // path
    Pose2d targetPose = isRed() ? targetTag.redPose : targetTag.bluePose;
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
                Math.abs(offsetX) < 0.02 && Math.abs(offsetY) < 0.02 && Math.abs(offsetAngle) < 1)
        .andThen(Commands.runOnce(() -> drive.stop()));
  }
}
