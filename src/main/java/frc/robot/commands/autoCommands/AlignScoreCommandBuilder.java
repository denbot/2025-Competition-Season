package frc.robot.commands.autoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.boathook.Boathook;
import frc.robot.subsystems.intake.Intake;
import java.util.ArrayList;

public class AlignScoreCommandBuilder {
  private Boathook boathook;
  private Intake intake;
  private ArrayList<Command> commands = new ArrayList<>();

  public AlignScoreCommandBuilder(Boathook boathook, Intake intake) {
    this.boathook = boathook;
    this.intake = intake;
  }

  public AlignScoreCommandBuilder reset() {
    return new AlignScoreCommandBuilder(boathook, intake);
  }

  public Command build(boolean clearCommands) {
    SequentialCommandGroup buildGroup = new SequentialCommandGroup();
    for (Command c : commands) {
      buildGroup.addCommands(c);
    }
    if(clearCommands) commands.clear();
    return buildGroup;
  }

  public AlignScoreCommandBuilder addAutoAlign(OnTheFlyTargetPose targetPose) {
    commands.add(getAutoAlignCommand(targetPose));
    return this;
  }

  public AlignScoreCommandBuilder addScoreL1() {
    commands.add(setIntakeAngle(IntakeConstants.intakeL1Angle, 0, 0));
    return this;
  }

  public AlignScoreCommandBuilder addScoreL2() {
    commands.add(setLengthCommand(0.02));
    commands.add(setAngleCommand(93));
    commands.add(setLengthCommand(0.85));
    commands.add(setAngleCommand(115));
    commands.add(new WaitCommand(2));
    commands.add(setAngleCommand(133));
    commands.add(setLengthCommand(0.02));
    commands.add(setAngleCommand(93));
    return this;
  }

  public AlignScoreCommandBuilder addScoreL3() {
    commands.add(setLengthCommand(0.02));
    commands.add(setAngleCommand(93));
    commands.add(setLengthCommand(1.95));
    commands.add(setAngleCommand(108));
    commands.add(new WaitCommand(2));
    commands.add(setAngleCommand(115));
    commands.add(setLengthCommand(0.02));
    commands.add(setAngleCommand(93));
    return this;
  }

  public AlignScoreCommandBuilder addScoreL4() {
    commands.add(setLengthCommand(0.02));
    commands.add(setAngleCommand(91));
    commands.add(setLengthCommand(4.3));
    commands.add(setAngleCommand(97));
    commands.add(new WaitCommand(2));
    commands.add(setLengthCommand(1.95));
    commands.add(setAngleCommand(93));
    commands.add(setAngleCommand(0.02));
    return this;
  }

  public AlignScoreCommandBuilder addIntakePiece() {
    // (TODO: Add vision based intake implementation)
    commands.add(toggleIntakeOnCommand());
    commands.add(new WaitCommand(1));
    commands.add(toggleIntakeOffCommand());
    return this;
  }

  public AlignScoreCommandBuilder addHandoff() {
    commands.add(setAngleCommand(93));
    commands.add(setLengthCommand(0.02));
    commands.add(setAngleCommand(35));
    commands.add(setIntakeAngle(IntakeConstants.intakeSpearAngle, 2, 0));
    commands.add(toggleIntakeReverseCommand());
    commands.add(setAngleCommand(93));
    commands.add(toggleIntakeOffCommand());
    commands.add(setIntakeAngle(IntakeConstants.intakeL1Angle, 1, 2));
    commands.add(setIntakeAngle(IntakeConstants.intakeDownAngle, 0, -3));
    return this;
  }

  public void clearCommands() {
    this.commands.clear();
  }

  public void removeLastCommand() {
    commands.remove(commands.size() - 1);
  }

  private Command getAutoAlignCommand(OnTheFlyTargetPose targetPose) {
    double x = targetPose.x;
    double y = targetPose.y;
    double angle = targetPose.angle;
    // if the alliance is red, flip positions accordingly
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      // approximate location of top right corner of the reef = 17.6, 7.6
      x = 17.6 - x;
      y = 8.05 - y;
      angle += 180;
      if (angle > 180) angle -= 360;
    }

    // initializes new pathFindToPose command which both create a path and has the robot follow said
    // path
    return AutoBuilder.pathfindToPose(
        new Pose2d(x, y, new Rotation2d(Units.degreesToRadians(angle))),
        new PathConstraints(2.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));
  }

  private Command toggleIntakeOnCommand() {
    return Commands.runOnce(() -> intake.setIntakeSpeed(IntakeConstants.intakeSpeed));
  }

  private Command toggleIntakeOffCommand() {
    return Commands.runOnce(() -> intake.setIntakeSpeed(0));
  }

  private Command toggleIntakeReverseCommand() {
    return Commands.runOnce(() -> intake.setIntakeSpeed(-IntakeConstants.intakeSpeed));
  }

  private Command setIntakeAngle(double angle, int slot, double feedForward) {
    return Commands.runOnce(() -> intake.setAngle(angle, slot, feedForward));
  }

  private Command setAngleCommand(double angle) {
    return Commands.runOnce(() -> boathook.setAngle(angle))
        .until(() -> (angle - boathook.getAngle() < 5));
  }

  private Command setLengthCommand(double length) {
    return Commands.runOnce(() -> boathook.setLength(length))
        .until(() -> (length - boathook.getLength() < 5));
  }
}
