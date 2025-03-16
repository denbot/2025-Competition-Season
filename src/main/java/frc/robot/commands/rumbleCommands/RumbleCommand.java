package frc.robot.commands.rumbleCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.commands.rumbleCommands.builders.RumbleScaleInterpolator;
import frc.robot.subsystems.RumbleSubsystem;

public class RumbleCommand extends Command {
  private final CommandGenericHID controller;
  private final RumbleScaleInterpolator leftInterpolator;
  private final RumbleScaleInterpolator rightInterpolator;
  private final Timer timer;

  public RumbleCommand(
      RumbleSubsystem rumbleSubsystem,
      RumbleScaleInterpolator leftInterpolator,
      RumbleScaleInterpolator rightInterpolator) {
    this.controller = rumbleSubsystem.controller;
    this.leftInterpolator = leftInterpolator;
    this.rightInterpolator = rightInterpolator;
    this.timer = new Timer();

    addRequirements(rumbleSubsystem);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(leftInterpolator.runTime)
        && timer.hasElapsed(rightInterpolator.runTime);
  }

  @Override
  public void execute() {
    controller.setRumble(
        GenericHID.RumbleType.kLeftRumble, leftInterpolator.rumbleAtTime(timer.get()));
    controller.setRumble(
        GenericHID.RumbleType.kLeftRumble, rightInterpolator.rumbleAtTime(timer.get()));
  }
}
