package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

/**
 * This class exists to prevent more than one rumble command running on the controller at any given
 * time.
 */
public class RumbleSubsystem extends SubsystemBase {
  public final CommandGenericHID controller;

  public RumbleSubsystem(CommandGenericHID controller) {
    this.controller = controller;
  }
}
