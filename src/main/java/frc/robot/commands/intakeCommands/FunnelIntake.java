// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FunnelIntake extends Command {
  /** Creates a new FunnelIntake. */
  Intake intake;

  double direction;

  public FunnelIntake(Intake intake, double direction) {
    addRequirements(intake);
    this.intake = intake;
    this.direction = direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setAngle(IntakeConstants.intakeFunnelAngle);
    intake.setIntakeSpeed(0);
    intake.setLeftIndexerSpeed(-IntakeConstants.indexerSpeed * direction);
    intake.setRightIndexerSpeed(IntakeConstants.indexerSpeed * direction);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("IntakeStatus", "Funneling");
    System.out.println("FUNNEL");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
