package frc.robot.commands.boathookCommands.setpointCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BoathookConstants;
import frc.robot.subsystems.boathook.Boathook;

public class BoathookAngleCommand extends Command {

  Boathook boathook;
  double angle;

  public BoathookAngleCommand(Boathook boathook, double angle) {
    this.boathook = boathook;
    addRequirements(boathook);
    this.angle = angle + boathook.microRotationOffset;
  }

  // Overload Constructor, command type
  public BoathookAngleCommand(Boathook boathook, int commandType) {
    this.boathook = boathook;
    addRequirements(boathook);

    switch (commandType) {
      case BoathookConstants.COMMAND_SET1:
        this.angle = boathook.getLevel().angle1;
        break;
      case BoathookConstants.COMMAND_SET2:
        this.angle = boathook.getLevel().angle2;
        break;
      case BoathookConstants.COMMAND_SET3:
        this.angle = boathook.getLevel().angle3;
        break;
      case BoathookConstants.COMMAND_IDLE:
        this.angle = BoathookConstants.IDLE_ANGLE;
        break;
      case BoathookConstants.COMMAND_STAB:
        this.angle = BoathookConstants.STAB_ANGLE;
        break;
      default:
        System.err.println("Command Type: " + commandType + " Not Understood");
        this.angle = boathook.getAngle();
        break;
    }
    this.angle += boathook.microRotationOffset;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boathook.setAngle(this.angle);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(boathook.getAngle() - this.angle) < 5;
  }
}
