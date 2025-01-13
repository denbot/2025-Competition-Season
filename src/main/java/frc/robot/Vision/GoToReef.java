// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToReef extends Command {
  /** Creates a new GoToReef. */
  public GoToReef() {
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // find which april tag on the reef is closest

    int[] ids = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    boolean isInIDs = false;

    double targetID = LimelightHelpers.getFiducialID("");

    for (int id : ids) {
      if (id == ids[id]) {
        isInIDs = true;
      }
    }

    if (isInIDs == false) {
      isFinished();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // move to either left or right, based on input given by controller
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
