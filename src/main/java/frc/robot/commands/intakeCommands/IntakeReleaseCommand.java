// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeReleaseCommand extends Command {
  /** Creates a new IndexReleaseCommand. */
  Intake intake;

  double time;
  Timer timer = new Timer();

  public IntakeReleaseCommand(Intake intake, double time) {
    addRequirements(intake);
    this.intake = intake;
    this.time = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset and start timer
    timer.reset();
    timer.start();
    System.out.println("intake release start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Reject piece
    intake.setIntakeSpeed(IntakeConstants.intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // When command is over, stop the intake wheels
    intake.setIntakeSpeed(0);
    System.out.println("intake release stop");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // After specified time has passed, end the command
    return timer.get() > time;
  }
}
