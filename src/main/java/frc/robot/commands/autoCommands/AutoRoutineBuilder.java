package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.ArrayList;

public class AutoRoutineBuilder {

  private SequentialCommandGroup autoRoutine;
  private ArrayList<String> commandStrings = new ArrayList<>();

  public AutoRoutineBuilder(BoathookCommands boathookCommands, IntakeCommands intakeCommands) {
    autoRoutine = new SequentialCommandGroup();
    autoRoutine.addCommands(boathookCommands.setBoathookIdle(), intakeCommands.intakeL1Command());
  }

  public AutoRoutineBuilder addPickupPieceBlock(Command pickupPieceCommand) {
    autoRoutine.addCommands(pickupPieceCommand);
    return this;
  }

  public String[] getCommandStrings() {
    String[] arr = new String[commandStrings.size()];
    for (int i = 0; i < commandStrings.size(); i++) {
      arr[i] = commandStrings.get(i);
    }
    return arr;
  }

  public AutoRoutineBuilder addBuildingBlock(Command autoAlign, Command scoreCommand) {
    autoRoutine.addCommands(autoAlign, scoreCommand);
    System.out.println("Added Command: " + autoAlign.getName() + ", " + scoreCommand.getName());
    commandStrings.add(autoAlign.getName() + ", " + scoreCommand.getName());
    return this;
  }

  public void clearCommands() {
    this.autoRoutine = new SequentialCommandGroup();
  }

  public SequentialCommandGroup build() {
    return autoRoutine;
  }
}
