// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.boathook.Boathook;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AngleBoathookCommand extends Command {
  Boathook boathook;
  double angle;
  /** Creates a new AngleBoathookCommand. */
  public AngleBoathookCommand(double angle, Boathook boathook) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.boathook = boathook;
    this.angle = angle;
    addRequirements(boathook);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boathook.setAngle(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(boathook.getAngle() - angle) < 1;
  }
}
