package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoRoutineBuilder {

  private SequentialCommandGroup autoRoutine;

  public AutoRoutineBuilder(BoathookCommands boathookCommands, IntakeCommands intakeCommands) {
    autoRoutine = new SequentialCommandGroup();
    // autoRoutine.addCommands(boathookCommands.setBoathookIdle(),
    // intakeCommands.intakeL1Command());
  }

  public AutoRoutineBuilder addPickupPieceBlock(Command pickupPieceCommand) {
    // autoRoutine.addCommands(pickupPieceCommand);
    return this;
  }

  public AutoRoutineBuilder addBuildingBlock(Command autoAlign, Command scoreCommand) {
    autoRoutine.addCommands(autoAlign /*, scoreCommand*/);
    return this;
  }

  public void clearCommands() {
    this.autoRoutine = new SequentialCommandGroup();
  }

  public SequentialCommandGroup build() {
    return autoRoutine;
  }
}
