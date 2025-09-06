package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.boathook.Boathook;
import java.util.function.BooleanSupplier;

public class BoathookCommands {

  private Boathook boathook;

  public BoathookCommands(Boathook boathook) {
    this.boathook = boathook;
  }

  public Command extendL2() {
    System.out.println("Extending L2");
    return new SequentialCommandGroup(
        setLengthCommand(0.4), setAngleCommand(93), setLengthCommand(0.85), setAngleCommand(115));
  }

  public Command retractL2() {
    System.out.println("Retracting L2");
    return new SequentialCommandGroup(
        setAngleCommand(133), setLengthCommand(0.4), setAngleCommand(93));
  }

  public Command extendL3() {
    System.out.println("Extending L3");
    return new SequentialCommandGroup(
        setLengthCommand(0.4), setAngleCommand(93), setLengthCommand(1.95), setAngleCommand(108));
  }

  public Command retractL3() {
    System.out.println("Retracting L3");
    return new SequentialCommandGroup(
        setAngleCommand(115), setLengthCommand(0.4), setAngleCommand(93));
  }

  public Command extendL4() {
    System.out.println("Extending L4");
    return new SequentialCommandGroup(
        setLengthCommand(0.4), setAngleCommand(91), setLengthCommand(4.3), setAngleCommand(97));
  }

  public Command retractL4() {
    System.out.println("Retracting L4");
    return new SequentialCommandGroup(
        setLengthCommand(1.95), setAngleCommand(93), setLengthCommand(0.4));
  }

  public Command setBoathookIdle() {
    System.out.println("Setting Boathook Idle");
    return new SequentialCommandGroup(setAngleCommand(93), setLengthCommand(0.4));
  }

  public Command setBoathookStab() {
    System.out.println("Setting Boathook Stab");
    return new SequentialCommandGroup(setLengthCommand(0.4), setAngleCommand(35));
  }

  public Command setAngleCommand(double angle) {
    System.out.println("Setting Angle To: " + angle);
    return (Commands.run(() -> boathook.setAngle(angle))).until(isAngleFinished());
  }

  public Command setLengthCommand(double length) {
    System.out.println("Setting Length To: " + length);
    return (Commands.run(() -> boathook.setLength(length))).until(isExtendFinished());
  }

  public BooleanSupplier isExtendFinished() {
    System.out.println("Length" + boathook.getLengthSetpoint() + ", " + boathook.getLength());
    return () -> (boathook.getLengthSetpoint() - boathook.getLength() < 0.1);
  }

  public BooleanSupplier isAngleFinished() {
    System.out.println("Angle" + boathook.getAngleSetpoint() + ", " + boathook.getAngle());
    return () -> (boathook.getAngleSetpoint() - boathook.getAngle() < 5);
  }
}
