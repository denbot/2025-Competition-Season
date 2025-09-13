package frc.robot.commands.visionCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.limelight.LimelightHelpers;
import frc.robot.util.limelight.LimelightHelpers.RawDetection;
import frc.robot.util.limelight.Limelights;

// object detection

// repositioning for intake

// movement for intake

public class AutomaticIntakeCommand extends Command {
  // creates a new AutomaticIntakeCommand
  DoublePublisher publisher;
  DoublePublisher anglePublisher;
  double kP = 5;
  double rotationalKP = 0.3;
  int framesDropped = 0;
  double angle = 0;
  double distance = 0;

  Drive drive;

  public AutomaticIntakeCommand(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
    publisher = NetworkTableInstance.getDefault().getDoubleTopic("Distance").publish();
    publisher.set(-3);
    anglePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Angle").publish();
  }

  public void initialize() {
    publisher.set(-2);
  }

  public void execute() {
    // if we drop a frame, do nothing for this periodic, unless we've dropped 6 or more frames, in
    // which case we end the command
    if (LimelightHelpers.getTV(Limelights.REAR.name)) {
      framesDropped = 0;
    } else {
      framesDropped++;
      return;
    }

    // gets raw data from the limelight
    RawDetection[] rawDetection = LimelightHelpers.getRawDetections("limelight-rear");

    if (rawDetection.length != 0) {
      angle = 90 + rawDetection[0].tync;
      distance = Math.tan(Math.toRadians(angle)) * 5.5625;
      publisher.set(distance);
      anglePublisher.set(angle);
    } else {
      publisher.set(-1);
    }

    double maxVelocity = 2; // TODO: When in large space set to 6
    double xDriveSpeed = Math.max(-maxVelocity, Math.min(maxVelocity, kP * Math.sin(distance)));
    SmartDashboard.putNumber("xDriveSpeed", xDriveSpeed);
    double yDriveSpeed = Math.max(-maxVelocity, Math.min(maxVelocity, kP * Math.cos(distance)));
    SmartDashboard.putNumber("yDriveSpeed", yDriveSpeed);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xDriveSpeed, yDriveSpeed, angle * rotationalKP);

    drive.runVelocity(chassisSpeeds);
  }

  public void end(boolean interrupted) {
    drive.stopWithX();
    SmartDashboard.putString("done", "done");
  }

  public boolean isFinished() {
    return (Math.abs(angle) < 3 && distance < 0.1) || (framesDropped > 5);
  }
}
