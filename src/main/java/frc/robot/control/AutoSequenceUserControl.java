package frc.robot.control;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.autoCommands.AutoRoutineBuilder;
import frc.robot.commands.autoCommands.OnTheFlyCommands;
import frc.robot.control.controllers.ButtonBoxController;
import frc.robot.game.ReefBranch;
import frc.robot.game.ReefLevel;
import frc.robot.subsystems.boathook.Boathook;
import frc.robot.subsystems.intake.Intake;

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
      Boathook boathook,
      Intake intake,
      OnTheFlyCommands onTheFlyCommands
  ) {
    this.autoRoutineBuilder = new AutoRoutineBuilder(boathook, intake, onTheFlyCommands);

    // Bind basic commands using our private event loop to ensure these setup actions
    // don't conflict with match-time behavior on the same buttons.
    buttonBoxController.lollipopUpTrigger(disabledEventLoop)
      .onTrue(Commands.runOnce(() -> autoRoutineBuilder.addPickupPieceBlock(
        onTheFlyCommands.pickupLollipop(
          ReefBranch.LOLLIPOP_UP_SETUP, 
          ReefBranch.LOLLIPOP_UP))
        ).ignoringDisable(true));
    buttonBoxController.lollipopCenterTrigger(disabledEventLoop)
      .onTrue(Commands.runOnce(() -> autoRoutineBuilder.addPickupPieceBlock(
        onTheFlyCommands.pickupLollipop(
          ReefBranch.LOLLIPOP_CENTER_SETUP, 
          ReefBranch.LOLLIPOP_CENTER))
        ).ignoringDisable(true));
    buttonBoxController.lollipopDownTrigger(disabledEventLoop)
      .onTrue(Commands.runOnce(() -> autoRoutineBuilder.addPickupPieceBlock(
        onTheFlyCommands.pickupLollipop(
          ReefBranch.LOLLIPOP_DOWN_SETUP, 
          ReefBranch.LOLLIPOP_DOWN))
      ).ignoringDisable(true));

    buttonBoxController.clearTrigger(disabledEventLoop).onTrue(Commands.runOnce(autoRoutineBuilder::clearCommands));

    var buttonToReefBranchMap = buttonBoxController.buttonToReefBranchMap(disabledEventLoop);
    var buttonToReefLevelMap = buttonBoxController.buttonToReefLevelMap(disabledEventLoop);

    // Cross-join the face and level triggers. This binds every combination of 
    // [Face Button] + [Level Button] so that pressing both simultaneously 
    // adds a specific "Building Block" (align + score) to our autonomous sequence.
    for (Map.Entry<Trigger, ReefBranch> face : buttonToReefBranchMap.entrySet()) {
      Trigger faceButton = face.getKey();
      ReefBranch targetBranch = face.getValue();

      for (Map.Entry<Trigger, ReefLevel> level : buttonToReefLevelMap.entrySet()) {
        Trigger levelButton = level.getKey();
        ReefLevel targetLevel = level.getValue();

        faceButton.and(levelButton)
            .onTrue(Commands.runOnce(() -> autoRoutineBuilder.addBuildingBlock(targetBranch, targetLevel))
            .ignoringDisable(true));
      }
    }
  }

  public Command runProgrammedSequence() {
    return Commands.run(() -> autoRoutineBuilder.build().schedule());
  }
}
