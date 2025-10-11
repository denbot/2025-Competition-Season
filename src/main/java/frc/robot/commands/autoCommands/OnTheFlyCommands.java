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
import frc.robot.Robot;
import frc.robot.commands.DriveCommands;

public class OnTheFlyCommands {

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
    LOLLIPOP_RIGHT_SETUP(2.5, 5.8, 180),
    LOLLIPOP_CENTER_SETUP(2.5, 4.0, 180),
    LOLLIPOP_LEFT_SETUP(2.5, 2.2, 180),
    LOLLIPOP_RIGHT(1.227, 5.8, 180),
    LOLLIPOP_CENTER(1.227, 4.0, 180),
    LOLLIPOP_LEFT(1.227, 2.2, 180);

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

  private static double currentRobotX;
  private static double currentRobotY;
  private static double currentRobotAngle;
  private static double offsetX;
  private static double offsetY;
  private static double offsetAngle;

  private static double translationalKP = 5;
  private static double angularKP = 0.5;

  public static Command alignTwoLeft() {
    return setCommandName(getAutoAlignCommand(OnTheFlyTargetPose.TWO_LEFT), "Align_Two_Left");
  }

  public static Command alignTwoRight() {
    return setCommandName(getAutoAlignCommand(OnTheFlyTargetPose.TWO_RIGHT), "Align_Two_Right");
  }

  public static Command alignFourLeft() {
    return setCommandName(getAutoAlignCommand(OnTheFlyTargetPose.FOUR_LEFT), "Align_Four_Left");
  }

  public static Command alignFourRight() {
    return setCommandName(getAutoAlignCommand(OnTheFlyTargetPose.FOUR_RIGHT), "Align_Four_Right");
  }

  public static Command alignSixLeft() {
    return setCommandName(getAutoAlignCommand(OnTheFlyTargetPose.SIX_LEFT), "Align_Six_Left");
  }

  public static Command alignSixRight() {
    return setCommandName(getAutoAlignCommand(OnTheFlyTargetPose.SIX_RIGHT), "Align_Six_Right");
  }

  public static Command alignEightLeft() {
    return setCommandName(getAutoAlignCommand(OnTheFlyTargetPose.EIGHT_LEFT), "Align_Eight_Left");
  }

  public static Command alignEightRight() {
    return setCommandName(getAutoAlignCommand(OnTheFlyTargetPose.EIGHT_RIGHT), "Align_Eight_Right");
  }

  public static Command alignTenLeft() {
    return setCommandName(getAutoAlignCommand(OnTheFlyTargetPose.TEN_LEFT), "Align_Ten_Left");
  }

  public static Command alignTenRight() {
    return setCommandName(getAutoAlignCommand(OnTheFlyTargetPose.TEN_RIGHT), "Align_Ten_Right");
  }

  public static Command alignTwelveLeft() {
    return setCommandName(getAutoAlignCommand(OnTheFlyTargetPose.TWELVE_LEFT), "Align_Twelve_Left");
  }

  public static Command alignTwelveRight() {
    return setCommandName(
        getAutoAlignCommand(OnTheFlyTargetPose.TWELVE_RIGHT), "Align_Twelve_Right");
  }

  public static Command pickupLollipopLeft(IntakeCommands intake) {
    return new SequentialCommandGroup(
        getAutoAlignCommand(OnTheFlyTargetPose.LOLLIPOP_LEFT_SETUP),
        intake.intakeDownCommand(),
        new ParallelCommandGroup(
            intake.runIntakeCommand().withTimeout(1),
            getAutoAlignCommand(OnTheFlyTargetPose.LOLLIPOP_LEFT)),
        Robot.robotContainer.boathookCommands.handoffCommand(intake, Robot.robotContainer.leds));
  }

  public static Command pickupLollipopRight(IntakeCommands intake) {
    return new SequentialCommandGroup(
        getAutoAlignCommand(OnTheFlyTargetPose.LOLLIPOP_RIGHT_SETUP),
        intake.intakeDownCommand(),
        new ParallelCommandGroup(
            intake.runIntakeCommand().withTimeout(1),
            getAutoAlignCommand(OnTheFlyTargetPose.LOLLIPOP_RIGHT)),
        Robot.robotContainer.boathookCommands.handoffCommand(intake, Robot.robotContainer.leds));
  }

  public static Command pickupLollipopCenter(IntakeCommands intake) {
    return new SequentialCommandGroup(
        getAutoAlignCommand(OnTheFlyTargetPose.LOLLIPOP_CENTER_SETUP),
        intake.intakeDownCommand(),
        new ParallelCommandGroup(
            intake.runIntakeCommand().withTimeout(1),
            getAutoAlignCommand(OnTheFlyTargetPose.LOLLIPOP_CENTER)),
        Robot.robotContainer.boathookCommands.handoffCommand(intake, Robot.robotContainer.leds));
  }

  private static Command getFinalAlignmentCommand(OnTheFlyTargetPose targetPose) {
    return Commands.runEnd(
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
              currentRobotX = Robot.robotContainer.drive.getPose().getX();
              currentRobotY = Robot.robotContainer.drive.getPose().getY();
              currentRobotAngle = Robot.robotContainer.drive.getPose().getRotation().getDegrees();
              offsetX = x - currentRobotX;
              offsetY = y - currentRobotY;
              offsetAngle = angle - currentRobotAngle;
              offsetAngle =
                  offsetAngle > 180
                      ? offsetAngle -= 360
                      : offsetAngle < -180 ? offsetAngle += 360 : offsetAngle;
              System.out.println("X: " + offsetX + "\nY: " + offsetY + "\nAngle: " + offsetAngle);
              ChassisSpeeds newSpeeds =
                  new ChassisSpeeds(
                      offsetX * translationalKP,
                      offsetY * translationalKP,
                      offsetAngle * angularKP);
              Robot.robotContainer.drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      newSpeeds, Robot.robotContainer.drive.getRotation()));
            },
            () ->
                DriveCommands.joystickDrive(
                    Robot.robotContainer.drive, () -> 0.0, () -> 0.0, () -> 0.0))
        .until(
            () ->
                Math.abs(offsetX) < 0.02 && Math.abs(offsetY) < 0.02 && Math.abs(offsetAngle) < 1);
  }

  private static Command getAutoAlignCommand(OnTheFlyTargetPose targetPose) {
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
            new PathConstraints(4.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)))
        .andThen(getFinalAlignmentCommand(targetPose));
  }

  private static Command setCommandName(Command command, String name) {
    command.setName(name);
    return command;
  }
}
