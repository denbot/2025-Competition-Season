// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.boathook.Boathook;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BoathookMotionPath extends SequentialCommandGroup {
  /** Creates a new BoathookMotionPath. */
  public BoathookMotionPath(Boathook boathook) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AngleBoathookCommand(boathook.angle1, boathook),
        new ExtendBoathookCommand(boathook.length1, boathook),
        new AngleBoathookCommand(boathook.angle2, boathook),
        new ExtendBoathookCommand(boathook.length2, boathook),
        new AngleBoathookCommand(boathook.angle3, boathook),
        new ExtendBoathookCommand(boathook.length3, boathook),
        new AngleBoathookCommand(90, boathook),
        new ExtendBoathookCommand(2, boathook));
  }
}
