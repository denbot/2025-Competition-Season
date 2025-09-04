// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.boathookCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.boathook.Boathook.Level;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetLevelCommand extends Command {
  /** Creates a new SetLevel. */
  private final Level level;

  public SetLevelCommand(Level level) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.level = level;
  }

  public SetLevelCommand getNew() {
    return new SetLevelCommand(this.level);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.robotContainer.boathook.setLevel(level);
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
