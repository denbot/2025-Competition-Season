package frc.robot.commands.autoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;

public class OnTheFlyAlignCommand extends Command {
  public Command pathFindingCommand;
  public Pose2d targetPose;

  public OnTheFlyAlignCommand(Drive drive) {
    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    double x = RobotContainer.currentTargetPose.x;
    double y = RobotContainer.currentTargetPose.y;
    double angle = RobotContainer.currentTargetPose.angle;
    // if the alliance is red, flip positions accordingly
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      // approximate location of top right corner of the reef = 17.6, 7.6
      x = 17.6 - x;
      y = 8.05 - y;
      angle += 180;
      if (angle > 180) angle -= 360;
    }
    System.out.println("Aligning To: " + x + ", " + y);

    // initializes new pathFindToPose command which both create a path and has the robot follow said
    // path
    pathFindingCommand =
        AutoBuilder.pathfindToPose(
            new Pose2d(x, y, new Rotation2d(Units.degreesToRadians(angle))),
            new PathConstraints(
                2.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));
  }

  @Override
  public void end(boolean interrupted) {
    // schedule the new command so it will actually move the robot
    pathFindingCommand.schedule();
    System.out.println("Ended On-The-Fly Command");
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
