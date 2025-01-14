// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.vision.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToReefCommand extends Command {
  /** Creates a new GoToReef. */

  // reef ids
  int[] ids = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

  int[] blueIDs = {17, 18, 19, 20, 21, 22};
  int[] redIDs = {6, 7, 8, 9, 10, 11};
  boolean isInIDs = false;

  double kP = 1.2;
  double rotationalKP = -0.05;

  double targetID;
  int differentTag = 0;

  boolean left;

  int framesDropped = 0;

  public GoToReefCommand(boolean right) {
    // Use addRequirements() here to declare subsystem dependencies.
    left = !right;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    differentTag = 0;
    // find which april tag on the reef is closest

    // which target we are looking at
    targetID = LimelightHelpers.getFiducialID("");

    // is the target we are looking at on the reef?
    for (int id : blueIDs) {
      // if we are on the red alliance, only look at the red ids
      if (Robot.getRed() && Robot.getTrust()) {
        if (targetID == redIDs[id]) {
          isInIDs = true;
        }
      }
      // if we are on the blue alliance, only look at the blue ids
      else if (Robot.getBlue() && Robot.getTrust()) {
        if (targetID == blueIDs[id]) {
          isInIDs = true;
        }
      }
    
      // if we don't know the alliance, we'll at least go to one of the reefs, we just might go to
      // the wrong one
      else {
        if (targetID == ids[id] || targetID == ids[id + 6]) {
          isInIDs = true;
        }
      }
    }

    if (isInIDs == false) {
      this.cancel();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getFiducialID("") == targetID) {
      differentTag = 0;
    } else {
      differentTag++;
      if (differentTag > 5) {
        this.cancel();
      }
      return;
    }
    
    // move to either left or right, based on input given by controller

    double[] tagPoseRobot = LimelightHelpers.getTargetPose_RobotSpace("");

    // converts the double array into Pose3d so we can use the values
    Pose3d pose =
        new Pose3d(
            new Translation3d(tagPoseRobot[0], tagPoseRobot[1], tagPoseRobot[2]),
            new Rotation3d(
                Math.toRadians(tagPoseRobot[3]),
                Math.toRadians(tagPoseRobot[4]),
                Math.toRadians(tagPoseRobot[5])));

    // if we drop a frame, do nothing for this periodic, unless we've dropped 6 or more frames, in
    // which case we end the command
    if (LimelightHelpers.getTV("")) {
      framesDropped = 0;
    } else {
      framesDropped++;
      if (framesDropped > 5) {
        this.cancel();
      }
      return;
    }

    // makes a Translation 3d object with our desired location relative to the april tag
    // then rotates and translates the translation so it is relative to the robot
    // at least thats what I think we are doing, I might have it wrong

    Translation3d translate;

    double leftOffset = 0.1651;

    if (left) {
      translate = new Translation3d(leftOffset, 0, 0);
    } else {
      translate = new Translation3d(-leftOffset, 0, 0);
    }

    translate = translate.rotateBy(pose.getRotation());
    translate = translate.plus(pose.getTranslation());

    double maxVelocity = 2; // TODO: When in large space set to 6
    double xDriveSpeed = Math.max(-maxVelocity, Math.min(maxVelocity, kP * translate.getZ()));
    double yDriveSpeed = Math.max(-maxVelocity, Math.min(maxVelocity, kP * translate.getX()));

    // TODO: Add the part that actually moves the robot
    ChassisSpeeds chassisSpeeds =
        new ChassisSpeeds(xDriveSpeed, yDriveSpeed, LimelightHelpers.getTX("") * rotationalKP);

    Robot.robotContainer.drive.runVelocity(chassisSpeeds);

    if (LimelightHelpers.getTX("") < 5
        && Math.sqrt(Math.pow(translate.getZ(), 2) + Math.pow(translate.getX(), 2)) < 0.25) {
      this.cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
