// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.visionCommands;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowOnTheFlyPath extends Command {
  /** Creates a new FollowOnTheFlyPath. */
  int face;

  List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
    new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)), 
    new Pose2d(1.0, 3.0, Rotation2d.fromDegrees(0)), 
    new Pose2d(3.0, 3.0, Rotation2d.fromDegrees(90))
  );

  PathConstraints constraints = new PathConstraints(6.0, 6.0, 2 * Math.PI, 4 * Math.PI);
  PathPlannerPath path;

  Drive drive;

  public FollowOnTheFlyPath(int reefFace) { // Use clock faces, so the furthest is 12, then 2, 4, 6, 8, 10
    // Use addRequirements() here to declare subsystem dependencies.
    face = reefFace;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (face) {
      case 12:
        // TODO set the path
        break;
      case 2:
        // TODO set the path
        break;
      case 4:
        // TODO set the path
        break;
      case 6:
        // TODO set the path
        break;
      case 8:
        // TODO set the path
        break;
      case 10:
        // TODO set the path
        break;
      default:
        this.cancel();
        break;
      
    }

    PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, Rotation2d.fromDegrees(-90)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.followPathCommand(path);
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
