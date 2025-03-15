// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.visionCommands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrajectoryTestCommand extends Command {
  /** Creates a new TrajectoryTestCommand. */
  Trajectory trajectoryTest;

  HolonomicDriveController controller;
  Timer timer = new Timer();
  double duration;

  Drive drive;

  public TrajectoryTestCommand(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(drive);
  }

  public void generateTrajectory() {
    // starting and ending poses, TO DO: change the poses to be more realistic
    Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    Pose2d endPose = new Pose2d(4, 4, Rotation2d.fromDegrees(90));

    ArrayList waypoints = new ArrayList<Pose2d>();
    waypoints.add(startPose);
    waypoints.add(endPose);

    // TO DO: change the configs to be what we actually want
    TrajectoryConfig config = new TrajectoryConfig(6, 6);
    config.setReversed(false);

    // Makes a new trajectory
    trajectoryTest = TrajectoryGenerator.generateTrajectory(waypoints, config);

    // just coppied wpi, so def will want to tune these values
    controller =
        new HolonomicDriveController(
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14)));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    duration = trajectoryTest.getTotalTimeSeconds();

    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State goal = trajectoryTest.sample(timer.get());

    ChassisSpeeds adjustedSpeeds =
        controller.calculate(drive.getPose(), goal, Rotation2d.fromDegrees(0.0));

    drive.runVelocity(adjustedSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > duration;
  }
}
