// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.elastic.Elastic;
import frc.robot.util.limelight.LimelightHelpers;
import frc.robot.util.limelight.LimelightPipeline;
import frc.robot.util.limelight.Limelights;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  public static RobotContainer robotContainer;
  private final Matrix<N3, N1> visionMatrix = new Matrix<>(Nat.N3(), Nat.N1());
  private Field2d field = new Field2d();
  private final Timer timer = new Timer();

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    SmartDashboard.putData("Field", field);
    // From
    // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization
    visionMatrix.fill(0.5); // X/Y location to 0.5
    visionMatrix.set(2, 0, 1); // Vision rotation is not to be trusted, apparently
  }

  @Override
  public void robotInit() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    Limelights.LEFT.setPipeline(LimelightPipeline.APRIL_TAG);
    Limelights.RIGHT.setPipeline(LimelightPipeline.APRIL_TAG);
    Limelights.REAR.setPipeline(LimelightPipeline.APRIL_TAG);

    robotContainer.preCheckTab.schedule();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(CommandScheduler.getInstance());
    // SmartDashboard.putNumber("Target X", RobotContainer.currentTargetPose.x);
    // SmartDashboard.putNumber("Target Y", RobotContainer.currentTargetPose.y);

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);
    maybeAddVisionMeasurement(Limelights.LEFT.name);
    maybeAddVisionMeasurement(Limelights.RIGHT.name);
    field.setRobotPose(robotContainer.drive.getPose());

    SmartDashboard.putNumberArray(
        "Pose", LimelightHelpers.getTargetPose_RobotSpace(Limelights.LEFT.name));
  }

  protected void maybeAddVisionMeasurement(String limelightName) {
    SmartDashboard.putNumber("Time Since Last ", timer.get());

    LimelightHelpers.PoseEstimate botPoseEstimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

    if (botPoseEstimate == null) {
      return; // No limelight connection just yet
      // TODO Maybe alert if this happens for too long
    }

    if (botPoseEstimate.tagCount == 0) {
      return; // Don't add vision when we can't see a tag
    }

    // If we only have one tag and one fiducial, we might need to bail
    if (botPoseEstimate.tagCount == 1 && botPoseEstimate.rawFiducials.length == 1) {
      // Highly ambiguous for direction/location? Reject it.
      if (botPoseEstimate.rawFiducials[0].ambiguity > .7) // TODO Tune this for our case
      {
        return;
      }

      // More than 3 meters away? Reject it.
      if (botPoseEstimate.rawFiducials[0].distToCamera > 3) {
        return;
      }
    }

    timer.restart();
    robotContainer.drive.addVisionMeasurement(
        limelightName, botPoseEstimate.pose, botPoseEstimate.timestampSeconds, visionMatrix);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    Elastic.Tabs.PRE_CHECK.show();
    robotContainer.preCheckTab.schedule();

    // Enable limelight 4 throttling when disabled to prevent overheating.
    LimelightHelpers.setThrottle("limelight-rear", 120);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    if (!DriverStation.getAlliance().isPresent())
      robotContainer.leds.solidInSectionCenter(0, 0, 255);
    else if (DriverStation.getAlliance().get() == Alliance.Red)
      robotContainer.leds.solidInSectionCenter(0, 255, 255);
    else robotContainer.leds.solidInSectionCenter(120, 255, 255);

    if (LimelightHelpers.getTV("limelight-right"))
      robotContainer.leds.solidInSection(0, 7, 60, 255, 255);
    else robotContainer.leds.solidInSection(0, 7, 0, 0, 0);
    if (LimelightHelpers.getTV("limelight-left"))
      robotContainer.leds.solidInSection(14, 21, 60, 255, 255);
    else robotContainer.leds.solidInSection(14, 21, 0, 0, 0);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Elastic.Tabs.AUTONOMOUS.show();

    // Disable limelight 4 throttling
    LimelightHelpers.setThrottle("limelight-rear", 0);

    autonomousCommand = robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    Elastic.Tabs.TELEOPERATED.show();

    // Disable limelight 4 throttling
    LimelightHelpers.setThrottle("limelight-rear", 0);

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Gyro", robotContainer.drive.getRotation().getDegrees());
    // SmartDashboard.putString(
    //     "Direction", String.valueOf(RobotContainer.currentTargetPose.direction));
    // SmartDashboard.putNumber("Angle", RobotContainer.currentTargetPose.angle);
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Disable limelight 4 throttling
    LimelightHelpers.setThrottle("limelight-rear", 0);

    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    // Enable limelight 4 throttling to prevent overheating.
    LimelightHelpers.setThrottle("limelight-rear", 120);
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public static RumblePresets rumble() {
    return robotContainer.rumblePresets;
  }
}
