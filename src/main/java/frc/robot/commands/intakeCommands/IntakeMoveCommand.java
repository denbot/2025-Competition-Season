// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeMoveCommand extends Command {
  /** Creates a new StopIntake. */
  Intake intake;

  boolean toggleEnable;

  public IntakeMoveCommand(Intake intake, boolean toggleEnable) {
    addRequirements(intake);
    this.intake = intake;
    this.toggleEnable = toggleEnable;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //If we are in teleop, the same button will toggle the intake up or down
    //Otherwise, we are in auto and just want the angle set to a specific place.
    if (toggleEnable) {
      intake.flipUp();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Tell the drive team if the intake should be up or down and set the angle
    SmartDashboard.putBoolean("IntakeAtL1", intake.up);
    intake.setAngle(intake.up ? IntakeConstants.intakeL1Angle : IntakeConstants.intakeDownAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
