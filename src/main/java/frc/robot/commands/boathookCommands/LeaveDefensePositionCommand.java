// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.boathookCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.boathookCommands.setpointCommands.AngleIdleBoathookCommand;
import frc.robot.commands.intakeCommands.HandoffPrepIntakeCommand;
import frc.robot.commands.intakeCommands.IntakeMoveCommand;
import frc.robot.subsystems.boathook.Boathook;
import frc.robot.subsystems.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeaveDefensePositionCommand extends SequentialCommandGroup {
  /** Creates a new BoathookMotionPath. */
  public LeaveDefensePositionCommand(Boathook boathook, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new HandoffPrepIntakeCommand(intake, 0.25),
        new ParallelCommandGroup(
            new AngleIdleBoathookCommand(boathook), new HandoffPrepIntakeCommand(intake, 0.5)),
        new IntakeMoveCommand(intake, false, IntakeConstants.intakeL1Angle, 1, 2),
        new IntakeMoveCommand(intake, false, IntakeConstants.intakeDownAngle, 0, -5));
  }
}
