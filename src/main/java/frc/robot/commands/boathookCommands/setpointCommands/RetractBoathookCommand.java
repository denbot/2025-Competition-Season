// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.boathookCommands.setpointCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.boathook.Boathook;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RetractBoathookCommand extends Command {
  Boathook boathook;
  double boathookAngle;
  double boathookLength;
  /** Creates a new AngleBoathookCommand. */
  public RetractBoathookCommand(Boathook boathook) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.boathook = boathook;
    addRequirements(boathook);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.boathookAngle = boathook.getLevel().angle3;
    this.boathookLength = boathook.getLevel().length2;
    System.out.println("New Boathook Command Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boathook.setAngle(boathookAngle);
    boathook.setLength(boathookLength);
    // System.out.println("CURRENT ANGLE: " + boathook.getAngle());
    // System.out.println("SET ANGLE: " + (boathookAngle));
    // System.out.println("SET POINT ANGLE: " + boathook.getAngleSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(boathook.getAngle() - (boathookAngle)) < 8
        && Math.abs(boathook.getLength() - boathookLength) < 0.1;
  }
}
