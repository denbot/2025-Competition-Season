// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.boathookCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.boathookCommands.setpointCommands.setBoathookStateCommand;
import frc.robot.commands.boathookCommands.setpointCommands.setBoathookStateCommand.boathookAngle;
import frc.robot.commands.boathookCommands.setpointCommands.setBoathookStateCommand.boathookLength;
import frc.robot.subsystems.boathook.Boathook;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BoathookExtendMotionPathCommand extends SequentialCommandGroup {
  /** Creates a new BoathookMotionPath. */
  public BoathookExtendMotionPathCommand(Boathook boathook) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new setBoathookStateCommand(
            boathook, boathookAngle.startAngle, boathookLength.noChangeLength),
        new setBoathookStateCommand(
            boathook, boathookAngle.noChangeAngle, boathookLength.startLength),
        new setBoathookStateCommand(
            boathook, boathookAngle.setupAngle, boathookLength.noChangeLength));
  }
}
