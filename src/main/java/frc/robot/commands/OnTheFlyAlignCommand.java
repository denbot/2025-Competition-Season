package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class OnTheFlyAlignCommand extends Command {
  private static PathConstraints constraints =
      new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  Command pathFindingCommand;
  public Pose2d targetPose;

  public OnTheFlyAlignCommand(Drive drive, Pose2d targetPose) {
    System.out.println("Created");
    this.targetPose = targetPose;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    System.out.println("Executing On-The-Fly Command");
    pathFindingCommand = AutoBuilder.pathfindToPose(this.targetPose, constraints);
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
