// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.boathookCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetSetPointsCommand extends Command {
  /** Creates a new SetSetPoints. */
  private final double angle1;

  private final double length1;
  private final double angle2;
  private final double length2;
  private final double angle3;
  private final double length3;

  public SetSetPointsCommand(
      double angle1, double length1, double angle2, double length2, double angle3, double length3) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle1 = angle1;
    this.length1 = length1;
    this.angle2 = angle2;
    this.length2 = length2;
    this.angle3 = angle3;
    this.length3 = length3;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.robotContainer.boathook.angle1 = angle1;
    Robot.robotContainer.boathook.length1 = length1;
    Robot.robotContainer.boathook.angle2 = angle2;
    Robot.robotContainer.boathook.length2 = length2;
    Robot.robotContainer.boathook.angle3 = angle3;
    Robot.robotContainer.boathook.length3 = length3;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

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
