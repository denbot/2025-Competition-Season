// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.visionCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Direction;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PipelineChange extends Command {
  /** Creates a new GoToReef. */
  int pipeline;

  double angle;
  Direction direction;

  public PipelineChange(int pipeline, Direction direction, double angle) {
    this.pipeline = pipeline;
    this.direction = direction;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.direction = this.direction;
    Robot.angle = this.angle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // sets the limelight pipeline to the desired side of the reef
    // the pipeline (which can be found in the limelight web ui) changes the accepted limelights
    // each pipeline is set to only accept one side of the reef
    LimelightHelpers.setPipelineIndex("limelight-left", pipeline);
    LimelightHelpers.setPipelineIndex("limelight-right", pipeline);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DONE");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
