// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.visionCommands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Direction;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToReefCommand extends Command {
  /** Creates a new GoToReef. */
  double kP = 5;

  double rotationalKP = 0.3;
  Direction direction = Direction.LEFT;
  int framesDropped = 0;
  Translation3d translate;
  double lastAngleError = 0;

  Drive drive;

  public GoToReefCommand(Drive drive) {
    this.direction = Robot.direction;
    this.drive = drive;
    // addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.direction = Robot.direction;
    SmartDashboard.putString("commandDirection", String.valueOf(direction));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if we drop a frame, do nothing for this periodic, unless we've dropped 6 or more frames, in
    // which case we end the command
    if (LimelightHelpers.getTV("limelight-left") || LimelightHelpers.getTV("limelight-right")) {
      framesDropped = 0;
    } else {
      framesDropped++;
      return;
    }

    // Used to "flip" the rotation of the application whenever the field is not blue
    var currentAlliance = DriverStation.getAlliance();
    double targetAngle = Robot.angle;
    if (currentAlliance.get() == Alliance.Red) {
      targetAngle += 180;
      if (targetAngle > 180) {
        targetAngle -= 360;
      }
    }

    // Determines the shortest angle error direction to correct for the angle wrap.
    lastAngleError = targetAngle - drive.getRotation().getDegrees();
    if (lastAngleError > 180) {
      lastAngleError -= 360;
    } else if (lastAngleError < -180) {
      lastAngleError += 360;
    }

    System.out.println(lastAngleError);

    double[] tagPoseRobot;
    // if in simulation, comment out this line:
    if (LimelightHelpers.getTV("limelight-left")) {
      tagPoseRobot = LimelightHelpers.getTargetPose_RobotSpace("limelight-left");

    } else {
      tagPoseRobot = LimelightHelpers.getTargetPose_RobotSpace("limelight-right");
    }

    // comment this line out before actually running the robot:
    // double[] tagPoseRobot = {0, 0, 0};

    // converts the double array into Pose3d so we can use the values
    Pose3d pose =
        new Pose3d(
            new Translation3d(tagPoseRobot[0], tagPoseRobot[1], tagPoseRobot[2]),
            new Rotation3d(
                Math.toRadians(tagPoseRobot[3]),
                Math.toRadians(tagPoseRobot[4]),
                Math.toRadians(tagPoseRobot[5])));

    // makes a Translation 3d object with our desired location relative to the april tag
    // then rotates and translates the translation so it is relative to the robot
    // at least thats what I think we are doing, I might have it wrong

    double offset = (direction == Direction.LEFT) ? -0.2 : 0.2;
    translate = new Translation3d(offset, 0, -0.5);

    translate = translate.rotateBy(pose.getRotation());
    translate = translate.plus(pose.getTranslation());

    double maxVelocity = 5; // TODO: When in large space set to 6
    double xDriveSpeed = Math.max(-maxVelocity, Math.min(maxVelocity, kP * translate.getZ()));
    SmartDashboard.putNumber("xDriveSpeed", xDriveSpeed);
    double yDriveSpeed = Math.max(-maxVelocity, Math.min(maxVelocity, kP * -translate.getX()));
    SmartDashboard.putNumber("yDriveSpeed", yDriveSpeed);

    ChassisSpeeds chassisSpeeds =
        new ChassisSpeeds(xDriveSpeed, yDriveSpeed, lastAngleError * rotationalKP);

    drive.runVelocity(chassisSpeeds);
    SmartDashboard.putNumber("error", lastAngleError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stopWithX();
    SmartDashboard.putString("done", "done");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(lastAngleError) < 3
            && Math.sqrt(Math.pow(translate.getZ(), 2) + Math.pow(translate.getX(), 2)) < 0.1)
        || (framesDropped > 5);
  }
}
