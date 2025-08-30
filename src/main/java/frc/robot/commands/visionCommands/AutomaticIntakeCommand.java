package frc.robot.commands.visionCommands;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.limelight.LimelightHelpers;
import frc.robot.util.limelight.LimelightHelpers.RawDetection;

// object detection

// repositioning for intake

// movement for intake

public class AutomaticIntakeCommand extends Command {
  // creates a new AutomaticIntakeCommand
  DoublePublisher publisher;
  DoublePublisher anglePublisher;

  public AutomaticIntakeCommand() {
    publisher = NetworkTableInstance.getDefault().getDoubleTopic("Distance").publish();
    publisher.set(-3);
    anglePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Angle").publish();
  }

  public void initialize() {
    publisher.set(-2);
  }

  public void execute() {
    // gets raw data from the limelight
    RawDetection[] rawDetection = LimelightHelpers.getRawDetections("limelight-rear");

    if (rawDetection.length != 0) {
      double angle = 90 + rawDetection[0].tync;
      double distance = Math.tan(Math.toRadians(angle)) * 5.5625;
      publisher.set(distance);
      anglePublisher.set(angle);
    } else {
      publisher.set(-1);
    }
  }

  public void end(boolean interrupted) {}

  public boolean isFinished() {
    return false;
  }
}
