package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class exists to prevent more than one rumble command running on the controller at any given
 * time.
 */
public class RumbleSubsystem extends SubsystemBase {
  public final XboxController controller;

  public RumbleSubsystem(XboxController controller) {
    this.controller = controller;
  }
}
