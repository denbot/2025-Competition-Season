// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.boathookCommands.setpointCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.boathook.Boathook;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MicroAdjustExtensionCommand extends Command {
  /** Creates a new MicroAdjustRotationCommand. */
  private final Boathook boathook;

  public enum ExtensionDirection {
    OffsetOutwards(0.03),
    OffsetInwards(-0.03);

    private final double offsetIncrement;

    ExtensionDirection(double offsetIncrement) {
      this.offsetIncrement = offsetIncrement;
    }
  }

  private ExtensionDirection direction;

  public MicroAdjustExtensionCommand(Boathook boathook, ExtensionDirection direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.boathook = boathook;
    this.direction = direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boathook.setLength(boathook.getLengthSetpoint() + direction.offsetIncrement);
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
}
