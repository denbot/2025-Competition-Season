// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.visionCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Direction;
import frc.robot.Constants.ReefTarget;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetChange extends Command {
  /** Creates a new GoToReef. */
  private final double angle;

  private final Direction direction;
  private final ReefTarget target;

  public TargetChange(Direction direction, double angle, ReefTarget target) {
    this.direction = direction;
    this.angle = angle;
    this.target = target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.direction = this.direction;
    Robot.angle = this.angle;
    Robot.target = this.target;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
