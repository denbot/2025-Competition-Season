package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.boathook.Boathook;
import frc.robot.subsystems.intake.Intake;

import java.util.ArrayList;

public class AutoRoutineBuilder {

  private SequentialCommandGroup autoRoutine;
  private ArrayList<String> commandStrings = new ArrayList<>();
  private Boathook boathook;
  private Intake intake;

  public AutoRoutineBuilder(Boathook boathook, Intake intake) {
    autoRoutine = new SequentialCommandGroup();
    this.boathook = boathook;
    this.intake = intake;
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
    autoRoutine.addCommands(
        new ParallelCommandGroup(
            autoAlign, boathook.setBoathookIdle(), intake.intakeL1Command()),
        scoreCommand);
    SmartDashboard.putString("Added Command", autoAlign.getName() + ", " + scoreCommand.getName());
    commandStrings.add(autoAlign.getName() + ", " + scoreCommand.getName());
    return this;
  }

  public void clearCommands() {
    this.autoRoutine = new SequentialCommandGroup();
    this.autoRoutine.addCommands(
        this.boathook.setBoathookIdle(), this.intake.intakeL1Command());
    SmartDashboard.putString("Added Command", "Cleared Commands");
  }

  public SequentialCommandGroup build() {
    return autoRoutine;
  }
}
