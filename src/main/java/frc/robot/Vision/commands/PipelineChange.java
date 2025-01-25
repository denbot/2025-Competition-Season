// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PipelineChange extends Command {
  /** Creates a new GoToReef. */
  int pipeline;

  boolean left;

  public PipelineChange(int pipeline, boolean left) {
    this.pipeline = pipeline;
    this.left = left;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.left = this.left;
    System.out.println(left);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // sets the limelight pipeline to the desired side of the reef
    // the pipeline (which can be found in the limelight web ui) changes the accepted limelights
    // each pipeline is set to only accept one side of the reef

    LimelightHelpers.setPipelineIndex("", pipeline);
    this.cancel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DONE");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
