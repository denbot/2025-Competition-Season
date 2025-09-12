package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoRoutineBuilder {

  private SequentialCommandGroup autoRoutine;

  public AutoRoutineBuilder() {
    autoRoutine = new SequentialCommandGroup();
  }

  public AutoRoutineBuilder addPickupPieceBlock(Command pickupPieceCommand) {
    autoRoutine.addCommands(pickupPieceCommand);
    return this;
  }

  public AutoRoutineBuilder addBuildingBlock(Command autoAlign, Command scoreCommand) {
    autoRoutine.addCommands(autoAlign);
    return this;
  }

  public void clearCommands() {
    this.autoRoutine = new SequentialCommandGroup();
  }

  public SequentialCommandGroup build() {
    return autoRoutine;
  }
}
