package frc.robot.commands.autoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.game.ReefBranch;
import frc.robot.subsystems.boathook.Boathook;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;

import java.util.Arrays;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;

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
  private double[] branchDistances = new double[12];

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

  public Command pickupLollipop(ReefBranch setupTarget, ReefBranch intakeTarget) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            getAutoAlignCommand(setupTarget),
            intake.intakeDownCommand(),
            boathook.setBoathookStab()),
        new ParallelCommandGroup(
            intake.runIntakeCommand().withTimeout(2),
            getAutoAlignCommand(intakeTarget)),
        boathook.handoffCommand(intake));
  }

  public Command getAutoAlignCommand(ReefBranch targetTag) {
    // initializes new pathFindToPose command which both create a path and has the robot follow said
    // path
    Pose2d targetPose = isRed() ? targetTag.redScoringPose : targetTag.blueScoringPose;
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

  public Command orbitReef(
    // DoubleSupplier xSupplier,
    // DoubleSupplier ySupplier
  ) {
    return Commands.run(
            () -> {
              int i = 0;
              if(isRed()){
                for (ReefBranch branch : ReefBranch.values()) {
                  if(branch.orbitEnabled){
                    double dX = drive.getPose().getX() - branch.redScoringPose.getX();
                    double dY = drive.getPose().getY() - branch.redScoringPose.getY();
                    branchDistances[i] = Math.sqrt(dX*dX + dY*dY);
                    i++;
                  }
                }
              } else {
                for (ReefBranch branch : ReefBranch.values()) {
                  if(branch.orbitEnabled){
                    double dX = drive.getPose().getX() - branch.blueScoringPose.getX();
                    double dY = drive.getPose().getY() - branch.blueScoringPose.getY();
                    branchDistances[i] = Math.sqrt(dX*dX + dY*dY);
                    i++;
                  }
                }
              }
              for(int j = 0; j < branchDistances.length; j++){
                SmartDashboard.putNumber("Tag: " + ReefBranch.values()[j].name(), branchDistances[j]);
              }

              SmartDashboard.putString(
                "Shortest Distance: ", 
                minDistance(branchDistances).name()
              );
            })
        .alongWith();
  }

  private ReefBranch minDistance (double[] distances){
    double minValue = 100;
    int minIndex = 0;
    for (int i = 0; i < distances.length; i++) {
      // If the current element is smaller than the current minimum value
      if (distances[i] < minValue) {
          minValue = distances[i]; // Update the minimum value
          minIndex = i; // Update the index of the minimum value
      }
    }
    return ReefBranch.values()[minIndex];
  }
}
