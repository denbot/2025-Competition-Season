package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.game.ReefBranch;
import frc.robot.game.ReefLevel;
import frc.robot.subsystems.boathook.Boathook;
import frc.robot.subsystems.intake.Intake;

import java.util.ArrayList;

public class AutoRoutineBuilder {

  private SequentialCommandGroup autoRoutine;
  private ArrayList<String> commandStrings = new ArrayList<>();
  private Boathook boathook;
  private Intake intake;
  private OnTheFlyCommands onTheFlyCommands;

  public AutoRoutineBuilder(Boathook boathook, Intake intake, OnTheFlyCommands onTheFlyCommands) {
    autoRoutine = new SequentialCommandGroup();
    this.boathook = boathook;
    this.intake = intake;
    this.onTheFlyCommands = onTheFlyCommands;
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

  public AutoRoutineBuilder addBuildingBlock(ReefBranch targetBranch, ReefLevel targetLevel) {
    Command autoAlignCommand = onTheFlyCommands.getAutoAlignCommand(targetBranch);
    Command scoreCommand = boathook.getLevelCommand(targetLevel);
    autoRoutine.addCommands(
      new ParallelCommandGroup(
        autoAlignCommand, 
        boathook.setBoathookIdle(), 
        intake.intakeL1Command()),
        scoreCommand
    );
    commandStrings.add(autoAlignCommand.getName() + ", " + scoreCommand.getName());
    return this;
  }

  public void clearCommands() {
    this.autoRoutine = new SequentialCommandGroup();
  }

  public SequentialCommandGroup build() {
    return autoRoutine;
  }
}
