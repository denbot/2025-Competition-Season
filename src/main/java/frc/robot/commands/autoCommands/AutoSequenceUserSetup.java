package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.ButtonBoxController;

import java.util.Map;

/**
 * A command that allows operators to program an autonomous routine using the physical button box
 * while the robot is disabled.
 *
 * <p>This command uses a dedicated internal {@link EventLoop} for its triggers. This allows the
 * physical buttons on the button box to have entirely different behaviors depending on which
 * loop is being polled. By using this private loop, we can ensure that these "setup" actions
 * are isolated from the "match" actions that will be bound to the same physical buttons
 * using a different event loop when the robot is enabled.
 *
 * <p>The programmed sequence is built via an {@link AutoRoutineBuilder} and can be retrieved
 * for execution during the autonomous period.
 */
public class AutoSequenceUserSetup extends Command {
  private final AutoRoutineBuilder autoRoutineBuilder;

  private final EventLoop eventLoop = new EventLoop();

  public AutoSequenceUserSetup(
      ButtonBoxController buttonBoxController,
      BoathookCommands boathookCommands,
      IntakeCommands intakeCommands,
      OnTheFlyCommands onTheFlyCommands
  ) {
    this.autoRoutineBuilder = new AutoRoutineBuilder(boathookCommands, intakeCommands);

    // Bind basic commands using our private event loop to ensure these setup actions
    // don't conflict with match-time behavior on the same buttons.
    buttonBoxController.lollipopLeftTrigger(eventLoop).onTrue(addPickupPieceBlock(onTheFlyCommands.pickupLollipopLeft()));
    buttonBoxController.lollipopCenterTrigger(eventLoop).onTrue(addPickupPieceBlock(onTheFlyCommands.pickupLollipopCenter()));
    buttonBoxController.lollipopRightTrigger(eventLoop).onTrue(addPickupPieceBlock(onTheFlyCommands.pickupLollipopRight()));

    buttonBoxController.spearTrigger(eventLoop).onTrue(Commands.runOnce(autoRoutineBuilder::clearCommands));

    // Define the possible face alignments and scoring levels available on the reef
    var reefFaceTriggers = Map.ofEntries(
        Map.entry(buttonBoxController.twelveLeftTrigger(eventLoop), onTheFlyCommands.alignTwelveLeft()),
        Map.entry(buttonBoxController.twelveRightTrigger(eventLoop), onTheFlyCommands.alignTwelveRight()),
        Map.entry(buttonBoxController.tenLeftTrigger(eventLoop), onTheFlyCommands.alignTenLeft()),
        Map.entry(buttonBoxController.tenRightTrigger(eventLoop), onTheFlyCommands.alignTenRight()),
        Map.entry(buttonBoxController.eightLeftTrigger(eventLoop), onTheFlyCommands.alignEightLeft()),
        Map.entry(buttonBoxController.eightRightTrigger(eventLoop), onTheFlyCommands.alignEightRight()),
        Map.entry(buttonBoxController.sixLeftTrigger(eventLoop), onTheFlyCommands.alignSixLeft()),
        Map.entry(buttonBoxController.sixRightTrigger(eventLoop), onTheFlyCommands.alignSixRight()),
        Map.entry(buttonBoxController.fourLeftTrigger(eventLoop), onTheFlyCommands.alignFourLeft()),
        Map.entry(buttonBoxController.fourRightTrigger(eventLoop), onTheFlyCommands.alignFourRight()),
        Map.entry(buttonBoxController.twoLeftTrigger(eventLoop), onTheFlyCommands.alignTwoLeft()),
        Map.entry(buttonBoxController.twoRightTrigger(eventLoop), onTheFlyCommands.alignTwoRight())
    );

    var reefLevelTriggers = Map.ofEntries(
        Map.entry(buttonBoxController.L4Trigger(eventLoop), boathookCommands.scoreL4()),
        Map.entry(buttonBoxController.L3Trigger(eventLoop), boathookCommands.scoreL3()),
        Map.entry(buttonBoxController.L2Trigger(eventLoop), boathookCommands.scoreL2())
    );

    // Cross-join the face and level triggers. This binds every combination of 
    // [Face Button] + [Level Button] so that pressing both simultaneously 
    // adds a specific "Building Block" (align + score) to our autonomous sequence.
    for (Map.Entry<Trigger, Command> face : reefFaceTriggers.entrySet()) {
      for (Map.Entry<Trigger, Command> level : reefLevelTriggers.entrySet()) {
        Command faceCommand = face.getValue();
        Command levelCommand = level.getValue();
        
        face.getKey().and(level.getKey())
            .onTrue(Commands.runOnce(() -> autoRoutineBuilder.addBuildingBlock(faceCommand, levelCommand)));
      }
    }
  }

  private Command addPickupPieceBlock(Command pickupPieceCommand) {
    return Commands.runOnce(() -> autoRoutineBuilder.addPickupPieceBlock(pickupPieceCommand));
  }

  @Override
  public boolean runsWhenDisabled() {
    return super.runsWhenDisabled();
  }

  @Override
  public void execute() {
    eventLoop.poll();

    SmartDashboard.putStringArray("Auto Routine List", autoRoutineBuilder.getCommandStrings());
  }

  @Override
  public boolean isFinished() {
    // As soon as we enable the robot, we can consider ourselves finished up here.
    return DriverStation.isEnabled();
  }

  public Command runProgrammedSequence() {
    return Commands.run(() -> {
      autoRoutineBuilder.build().schedule();
    });
  }
}
