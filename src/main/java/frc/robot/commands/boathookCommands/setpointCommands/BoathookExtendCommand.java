package frc.robot.commands.boathookCommands.setpointCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BoathookConstants;
import frc.robot.subsystems.boathook.Boathook;

/**
 * @param boathook Boathook Class Object To Apply The Command To
 * @param angle Length 1->3, idle, or Stab found from the BoathookConstants Class
 */
public class BoathookExtendCommand extends Command {

  Boathook boathook;
  double length;

  public BoathookExtendCommand(Boathook boathook, double length) {
    this.boathook = boathook;
    addRequirements(boathook);
    this.length = length;
  }

  // Overload Constructor, command type used to assign a length
  public BoathookExtendCommand(Boathook boathook, int commandType) {
    this.boathook = boathook;
    addRequirements(boathook);
    switch (commandType) {
      case BoathookConstants.COMMAND_SET1:
        this.length = boathook.getLevel().length1;
        break;
      case BoathookConstants.COMMAND_SET2:
        this.length = boathook.getLevel().length2;
        break;
      case BoathookConstants.COMMAND_SET3:
        this.length = boathook.getLevel().length3;
        break;
      case BoathookConstants.COMMAND_IDLE:
        this.length = BoathookConstants.IDLE_EXTENSION;
        break;
      case BoathookConstants.COMMAND_STAB:
        this.length = BoathookConstants.STAB_EXTENSION;
        break;
      default:
        System.err.println("Command Type: " + commandType + " Not Understood");
        this.length = boathook.getLength();
        break;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boathook.setLength(this.length);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    boathook.setBrakeExtender();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(boathook.getLength() - BoathookConstants.IDLE_EXTENSION) < 0.1);
  }
}
