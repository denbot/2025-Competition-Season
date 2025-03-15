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

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Direction;
import frc.robot.generated.TunerConstants;
import java.util.Optional;
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
  private Matrix<N3, N1> matrix = new Matrix<>(Nat.N3(), Nat.N1());
  public static Direction direction = Direction.LEFT;
  public static double angle;
  public static boolean red;
  private Field2d field = new Field2d();

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

    // Check for valid swerve config
    var modules =
        new SwerveModuleConstants[] {
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight
        };
    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
      }
    }

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    SmartDashboard.putData("Field", field);
    matrix.fill(1.0);
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

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);
    SmartDashboard.putNumber("RX", LimelightHelpers.getBotPose2d_wpiBlue("limelight-right").getX());
    SmartDashboard.putNumber("RY", LimelightHelpers.getBotPose2d_wpiBlue("limelight-right").getY());
    SmartDashboard.putNumber(
        "RZ", LimelightHelpers.getBotPose2d_wpiBlue("limelight-right").getRotation().getDegrees());
    SmartDashboard.putNumber("LX", LimelightHelpers.getBotPose2d_wpiBlue("limelight-left").getX());
    SmartDashboard.putNumber("LY", LimelightHelpers.getBotPose2d_wpiBlue("limelight-left").getY());
    SmartDashboard.putNumber(
        "LZ", LimelightHelpers.getBotPose2d_wpiBlue("limelight-left").getRotation().getDegrees());
    robotContainer.drive.addVisionMeasurement(
        "limelight-left",
        LimelightHelpers.getBotPose2d_wpiBlue("limelight-left"),
        Timer.getFPGATimestamp(),
        matrix);
    robotContainer.drive.addVisionMeasurement(
        "limelight-right",
        LimelightHelpers.getBotPose2d_wpiBlue("limelight-right"),
        Timer.getFPGATimestamp(),
        matrix);
    field.setRobotPose(robotContainer.drive.getPose());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // Enable limelight 4 throttling when disabled to prevent overheating.
    NetworkTableInstance.getDefault()
        .getTable("limelight-rear")
        .getEntry("throttle_set")
        .setNumber(120);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // Disable limelight 4 throttling
    NetworkTableInstance.getDefault()
        .getTable("limelight-rear")
        .getEntry("throttle_set")
        .setNumber(0);

    autonomousCommand = robotContainer.getAutonomousCommand();
    getRed();
    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  public static boolean getRed() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        red = true;
      }
      if (ally.get() == Alliance.Blue) {
        red = false;
      }
    }
    return red;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // Disable limelight 4 throttling
    NetworkTableInstance.getDefault()
        .getTable("limelight-rear")
        .getEntry("throttle_set")
        .setNumber(120);

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    getRed();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Gyro", robotContainer.drive.getRotation().getDegrees());
    SmartDashboard.putString("Direction", String.valueOf(direction));
    SmartDashboard.putNumber("Angle", angle);
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Disable limelight 4 throttling
    NetworkTableInstance.getDefault()
        .getTable("limelight-rear")
        .getEntry("throttle_set")
        .setNumber(0);

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
    NetworkTableInstance.getDefault()
        .getTable("limelight-rear")
        .getEntry("throttle_set")
        .setNumber(120);
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
