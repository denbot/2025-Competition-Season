package frc.robot.control;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.autoCommands.AutoRoutineBuilder;
import frc.robot.commands.autoCommands.BoathookCommands;
import frc.robot.commands.autoCommands.IntakeCommands;
import frc.robot.commands.autoCommands.OnTheFlyCommands;
import frc.robot.control.controllers.ButtonBoxController;

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
public class AutoSequenceUserControl {
  private final AutoRoutineBuilder autoRoutineBuilder;

  public AutoSequenceUserControl(
      EventLoop disabledEventLoop,
      ButtonBoxController buttonBoxController,
      BoathookCommands boathookCommands,
      IntakeCommands intakeCommands,
      OnTheFlyCommands onTheFlyCommands
  ) {
    this.autoRoutineBuilder = new AutoRoutineBuilder(boathookCommands, intakeCommands);

    // Bind basic commands using our private event loop to ensure these setup actions
    // don't conflict with match-time behavior on the same buttons.
    buttonBoxController.lollipopLeftTrigger(disabledEventLoop).onTrue(addPickupPieceBlock(onTheFlyCommands.pickupLollipopLeft()));
    buttonBoxController.lollipopCenterTrigger(disabledEventLoop).onTrue(addPickupPieceBlock(onTheFlyCommands.pickupLollipopCenter()));
    buttonBoxController.lollipopRightTrigger(disabledEventLoop).onTrue(addPickupPieceBlock(onTheFlyCommands.pickupLollipopRight()));

    buttonBoxController.spearTrigger(disabledEventLoop).onTrue(Commands.runOnce(autoRoutineBuilder::clearCommands));

    // Define the possible face alignments and scoring levels available on the reef
    var reefFaceTriggers = Map.ofEntries(
        Map.entry(buttonBoxController.twelveLeftTrigger(disabledEventLoop), onTheFlyCommands.alignTwelveLeft()),
        Map.entry(buttonBoxController.twelveRightTrigger(disabledEventLoop), onTheFlyCommands.alignTwelveRight()),
        Map.entry(buttonBoxController.tenLeftTrigger(disabledEventLoop), onTheFlyCommands.alignTenLeft()),
        Map.entry(buttonBoxController.tenRightTrigger(disabledEventLoop), onTheFlyCommands.alignTenRight()),
        Map.entry(buttonBoxController.eightLeftTrigger(disabledEventLoop), onTheFlyCommands.alignEightLeft()),
        Map.entry(buttonBoxController.eightRightTrigger(disabledEventLoop), onTheFlyCommands.alignEightRight()),
        Map.entry(buttonBoxController.sixLeftTrigger(disabledEventLoop), onTheFlyCommands.alignSixLeft()),
        Map.entry(buttonBoxController.sixRightTrigger(disabledEventLoop), onTheFlyCommands.alignSixRight()),
        Map.entry(buttonBoxController.fourLeftTrigger(disabledEventLoop), onTheFlyCommands.alignFourLeft()),
        Map.entry(buttonBoxController.fourRightTrigger(disabledEventLoop), onTheFlyCommands.alignFourRight()),
        Map.entry(buttonBoxController.twoLeftTrigger(disabledEventLoop), onTheFlyCommands.alignTwoLeft()),
        Map.entry(buttonBoxController.twoRightTrigger(disabledEventLoop), onTheFlyCommands.alignTwoRight())
    );

    var reefLevelTriggers = Map.ofEntries(
        Map.entry(buttonBoxController.L4Trigger(disabledEventLoop), boathookCommands.scoreL4()),
        Map.entry(buttonBoxController.L3Trigger(disabledEventLoop), boathookCommands.scoreL3()),
        Map.entry(buttonBoxController.L2Trigger(disabledEventLoop), boathookCommands.scoreL2())
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

  // TODO No execute anymore, but we will eventually need to give the user an indication for the auto routine's state
//  @Override
//  public void execute() {
//    SmartDashboard.putStringArray("Auto Routine List", autoRoutineBuilder.getCommandStrings());
//  }

  public Command runProgrammedSequence() {
    return Commands.run(() -> autoRoutineBuilder.build().schedule());
  }
}
