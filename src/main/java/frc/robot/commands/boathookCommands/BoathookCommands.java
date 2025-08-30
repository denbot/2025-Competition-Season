package frc.robot.commands.boathookCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BoathookConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.boathookCommands.setpointCommands.BoathookAngleCommand;
import frc.robot.commands.boathookCommands.setpointCommands.BoathookExtendCommand;
import frc.robot.commands.intakeCommands.HandoffPrepIntakeCommand;
import frc.robot.commands.intakeCommands.IntakeMoveCommand;
import frc.robot.subsystems.boathook.Boathook;
import frc.robot.subsystems.intake.Intake;

public class BoathookCommands {

  public static SequentialCommandGroup extendMotionPathCommand;
  public static SequentialCommandGroup idleCommand;
  public static SequentialCommandGroup retractMotionPathCommand;
  public static SequentialCommandGroup handoffCommand;

  // primarily instantiates versions of the commands for the provided boathook
  public BoathookCommands(Boathook boathook, Intake intake) {

    extendMotionPathCommand.addCommands(
        new BoathookAngleCommand(boathook, BoathookConstants.COMMAND_SET1),
        new BoathookExtendCommand(boathook, BoathookConstants.COMMAND_SET1),
        new BoathookAngleCommand(boathook, BoathookConstants.COMMAND_SET2));

    idleCommand.addCommands(new BoathookAngleCommand(boathook, BoathookConstants.COMMAND_IDLE));

    retractMotionPathCommand.addCommands(
        new BoathookExtendCommand(boathook, BoathookConstants.COMMAND_SET2),
        new BoathookAngleCommand(boathook, BoathookConstants.COMMAND_SET3),
        new BoathookExtendCommand(boathook, BoathookConstants.COMMAND_SET3),
        new BoathookAngleCommand(boathook, BoathookConstants.COMMAND_IDLE),
        new BoathookExtendCommand(boathook, BoathookConstants.COMMAND_IDLE));

    handoffCommand.addCommands(
        new BoathookAngleCommand(boathook, BoathookConstants.IDLE_ANGLE),
        new BoathookExtendCommand(boathook, BoathookConstants.STAB_EXTENSION),
        new BoathookAngleCommand(boathook, BoathookConstants.STAB_ANGLE),
        new IntakeMoveCommand(intake, false, IntakeConstants.intakeSpearAngle, 2, 0),
        new HandoffPrepIntakeCommand(intake, 0.25),
        new ParallelCommandGroup(
            new BoathookAngleCommand(boathook, BoathookConstants.IDLE_ANGLE),
            new HandoffPrepIntakeCommand(intake, 0.5)),
        new IntakeMoveCommand(intake, false, IntakeConstants.intakeL1Angle, 1, 2),
        new IntakeMoveCommand(intake, false, IntakeConstants.intakeDownAngle, 0, -3));
  }

  public static SequentialCommandGroup newExtendMotoinPathCommand(Boathook boathook) {
    return new SequentialCommandGroup(
        new BoathookAngleCommand(boathook, BoathookConstants.COMMAND_SET1),
        new BoathookExtendCommand(boathook, BoathookConstants.COMMAND_SET1),
        new BoathookAngleCommand(boathook, BoathookConstants.COMMAND_SET2));
  }

  public static SequentialCommandGroup newIdleCommand(Boathook boathook) {
    return new SequentialCommandGroup(
        new BoathookAngleCommand(boathook, BoathookConstants.COMMAND_IDLE));
  }

  public static SequentialCommandGroup newRetractMotionPathCommand(Boathook boathook) {
    return new SequentialCommandGroup(
        new BoathookExtendCommand(boathook, BoathookConstants.COMMAND_SET2),
        new BoathookAngleCommand(boathook, BoathookConstants.COMMAND_SET3),
        new BoathookExtendCommand(boathook, BoathookConstants.COMMAND_SET3),
        new BoathookAngleCommand(boathook, BoathookConstants.COMMAND_IDLE),
        new BoathookExtendCommand(boathook, BoathookConstants.COMMAND_IDLE));
  }

  public static SequentialCommandGroup newHandoffCommand(Boathook boathook, Intake intake) {
    return new SequentialCommandGroup(
        new BoathookAngleCommand(boathook, BoathookConstants.IDLE_ANGLE),
        new BoathookExtendCommand(boathook, BoathookConstants.STAB_EXTENSION),
        new BoathookAngleCommand(boathook, BoathookConstants.STAB_ANGLE),
        new IntakeMoveCommand(intake, false, IntakeConstants.intakeSpearAngle, 2, 0),
        new HandoffPrepIntakeCommand(intake, 0.25),
        new ParallelCommandGroup(
            new BoathookAngleCommand(boathook, BoathookConstants.IDLE_ANGLE),
            new HandoffPrepIntakeCommand(intake, 0.5)),
        new IntakeMoveCommand(intake, false, IntakeConstants.intakeL1Angle, 1, 2),
        new IntakeMoveCommand(intake, false, IntakeConstants.intakeDownAngle, 0, -3));
  }
}
