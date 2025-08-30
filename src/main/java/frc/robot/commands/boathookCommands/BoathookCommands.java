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

/**
 * Class encompassing functions to define and create new instances of SequentialCommandGroups
 * required to do tasks such as extend, retract, idle, or stab
 */
public class BoathookCommands {

  /**
   * Instantiates a new SequentialCommandGroup with the steps Set angle1, Set extend 1, Set angle2
   *
   * @param boathook
   * @return new SequentialCommandGroup
   */
  public static SequentialCommandGroup newExtendMotoinPathCommand(Boathook boathook) {
    return new SequentialCommandGroup(
        new BoathookAngleCommand(boathook, BoathookConstants.COMMAND_SET1),
        new BoathookExtendCommand(boathook, BoathookConstants.COMMAND_SET1),
        new BoathookAngleCommand(boathook, BoathookConstants.COMMAND_SET2));
  }

  /**
   * Instantiates a new SequentialCommandGroup that sets the angle to idle
   *
   * @param boathook
   * @return new SequentialCommandGroup
   */
  public static SequentialCommandGroup newIdleCommand(Boathook boathook) {
    return new SequentialCommandGroup(
        new BoathookAngleCommand(boathook, BoathookConstants.COMMAND_IDLE));
  }

  /**
   * Instantiates a new SequentialCommandGroup with the steps Set extend 2, Set angle3, Set extend
   * 3, set Angle idle, set Extend idle
   *
   * @param boathook
   * @return new SequentialCommandGroup
   */
  public static SequentialCommandGroup newRetractMotionPathCommand(Boathook boathook) {
    return new SequentialCommandGroup(
        new BoathookExtendCommand(boathook, BoathookConstants.COMMAND_SET2),
        new BoathookAngleCommand(boathook, BoathookConstants.COMMAND_SET3),
        new BoathookExtendCommand(boathook, BoathookConstants.COMMAND_SET3),
        new BoathookAngleCommand(boathook, BoathookConstants.COMMAND_IDLE),
        new BoathookExtendCommand(boathook, BoathookConstants.COMMAND_IDLE));
  }

  /**
   * Instantiates a new SequentialCommandGroup with the steps requeired for a proper intake-to-end
   * effector handoff
   *
   * @param boathook
   * @param intake
   * @return new SequentialCommandGroup
   */
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
