// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.boathookCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.boathookCommands.setpointCommands.Angle3BoathookCommand;
import frc.robot.commands.boathookCommands.setpointCommands.AngleIdleBoathookCommand;
import frc.robot.commands.boathookCommands.setpointCommands.ExtendBoathookCommand2;
import frc.robot.commands.boathookCommands.setpointCommands.ExtendBoathookCommand3;
import frc.robot.commands.boathookCommands.setpointCommands.ExtendBoathookCommandIdle;
import frc.robot.subsystems.boathook.Boathook;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BoathookRetractMotionPathCommand extends SequentialCommandGroup {
  /** Creates a new BoathookMotionPath. */
  public BoathookRetractMotionPathCommand(Boathook boathook) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ExtendBoathookCommand2(boathook),
        new Angle3BoathookCommand(boathook),
        new ExtendBoathookCommand3(boathook),
        new AngleIdleBoathookCommand(boathook));
    new ExtendBoathookCommandIdle(boathook);
  }
}
