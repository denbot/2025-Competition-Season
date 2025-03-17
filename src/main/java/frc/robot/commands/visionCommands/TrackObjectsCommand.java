// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.visionCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.vision.MultiCoralTracker;
import frc.robot.vision.MultiCoralTrackingState;
import frc.robot.vision.SingleCoralTracker;
import frc.robot.vision.SingleCoralTrackingState;
import java.util.ArrayList;

/** Example command that tracks objects. */
public class TrackObjectsCommand extends Command {
  private final SingleCoralTracker coralTracker = new SingleCoralTracker(1);
  private final MultiCoralTracker multiCoralTracker = new MultiCoralTracker(1);
  StructArrayPublisher<Translation2d> multiCoralTrackerPoints;
  StructArrayPublisher<Translation2d> singleCoralTrackerPoints;

  public TrackObjectsCommand() {
    multiCoralTrackerPoints =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("MultiCoralTracker", Translation2d.struct)
            .publish();

    singleCoralTrackerPoints =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("SingleCoralTracker", Translation2d.struct)
            .publish();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralTracker.reset();
    multiCoralTracker.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var detections = LimelightHelpers.getRawDetections("limelight-rear");

    //
    // SINGLE CORAL EXAMPLE
    //
    coralTracker.addRawDetections(detections);
    coralTracker.update();
    SingleCoralTrackingState singleTrackingState = coralTracker.getTrackingState();
    SmartDashboard.putBoolean("Object_Filter_Tracking", singleTrackingState.isTracking());
    SmartDashboard.putNumber("Object_Tx", singleTrackingState.tx());
    SmartDashboard.putNumber("Object_Ty", singleTrackingState.ty());
    SmartDashboard.putNumber("Object_Distance", singleTrackingState.getDistance());
    SmartDashboard.putBoolean("Object_Upright", singleTrackingState.isUpright());
    SmartDashboard.putNumber("Object_AspectRatio", singleTrackingState.getAspectRatio());

    // Publish single track points to display on advantge scope
    if (singleTrackingState.isTracking()) {
      singleCoralTrackerPoints.set(
          new Translation2d[] {
            new Translation2d(singleTrackingState.tx(), singleTrackingState.ty())
          });
    } else {
      singleCoralTrackerPoints.set(new Translation2d[] {new Translation2d(-100, -100)});
    }

    //
    // MULTI CORAL EXAMPLE
    // This currently results in the robot disconnecting.
    // I think it is because it is overrunning the 20ms loop time.
    //
    multiCoralTracker.addRawDetections(detections);
    multiCoralTracker.update();

    MultiCoralTrackingState multiTrackingState = multiCoralTracker.getTrackingState();

    SingleCoralTrackingState activeStates[] = multiTrackingState.getActiveTracks();
    SmartDashboard.putNumber("MultiObject_ActiveTracks", activeStates.length);

    // Publish multi track points to display on advantage scope
    ArrayList<Translation2d> multiTrackerPoints = new ArrayList<Translation2d>();

    for (int i = 0; i < multiTrackingState.states().length; i++) {
      if (multiTrackingState.states()[i].isTracking()) {
        multiTrackerPoints.add(
            new Translation2d(
                multiTrackingState.states()[i].tx(), multiTrackingState.states()[i].ty()));
      } else {
        multiTrackerPoints.add(new Translation2d(-100, -100));
      }
      SmartDashboard.putBoolean(
          "MultiObject" + i + "_Filter_Tracking", multiTrackingState.states()[i].isTracking());
    }
    multiCoralTrackerPoints.set(multiTrackerPoints.toArray(new Translation2d[0]));
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
