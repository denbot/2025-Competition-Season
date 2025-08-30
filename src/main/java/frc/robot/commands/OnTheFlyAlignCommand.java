package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;

public class OnTheFlyAlignCommand extends Command {
  // private static PathConstraints constraints;
  Command pathFindingCommand;
  public Pose2d targetPose;

  public OnTheFlyAlignCommand(Drive drive) {
    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    System.out.println(
        "Aligning To: "
            + RobotContainer.currentTargetPose.x
            + ", "
            + RobotContainer.currentTargetPose.y);
    pathFindingCommand =
        AutoBuilder.pathfindToPose(
            new Pose2d(
                RobotContainer.currentTargetPose.x,
                RobotContainer.currentTargetPose.y,
                new Rotation2d(Units.degreesToRadians(RobotContainer.currentTargetPose.angle))),
            new PathConstraints(
                0.5, 0.5, Units.degreesToRadians(540), Units.degreesToRadians(720)));
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Ended On-The-Fly Command");
    pathFindingCommand.schedule();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
