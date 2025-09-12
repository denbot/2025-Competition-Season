package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoRoutineBuilder {

  private SequentialCommandGroup autoRoutine;

  public AutoRoutineBuilder() {
    autoRoutine = new SequentialCommandGroup();
  }

  public AutoRoutineBuilder addBuildingBlock(Command autoAlign, Command scoreCommand) {
    System.out.println("Adding Building Block");
    autoRoutine.addCommands(autoAlign, scoreCommand);
    return this;
  }

  public SequentialCommandGroup build() {
    return autoRoutine;
  }
}
