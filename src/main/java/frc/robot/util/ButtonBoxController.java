package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBoxController {
  public static final CommandGenericHID controller1 = new CommandGenericHID(1);
  public static final CommandGenericHID controller2 = new CommandGenericHID(2);

  public Trigger L4Trigger() {
    return controller1.button(4);
  }

  public Trigger L3Trigger() {
    return controller1.button(5);
  }

  public Trigger L2Trigger() {
    return controller1.button(6);
  }

  public Trigger L1Trigger() {
    return controller1.button(7);
  }

  public Trigger spearTrigger() {
    return controller2.button(4);
  }

  public Trigger twoLeftTrigger() {
    return controller1.button(3);
  }

  public Trigger twoRightTrigger() {
    return controller1.button(2);
  }

  public Trigger fourLeftTrigger() {
    return controller1.button(11);
  }

  public Trigger fourRightTrigger() {
    return controller1.button(8);
  }

  public Trigger sixLeftTrigger() {
    return controller2.button(12);
  }

  public Trigger sixRightTrigger() {
    return controller1.button(12);
  }

  public Trigger eightLeftTrigger() {
    return controller2.button(8);
  }

  public Trigger eightRightTrigger() {
    return controller2.button(11);
  }

  public Trigger tenLeftTrigger() {
    return controller2.button(2);
  }

  public Trigger tenRightTrigger() {
    return controller2.button(3);
  }

  public Trigger twelveLeftTrigger() {
    return controller1.button(1);
  }

  public Trigger twelveRightTrigger() {
    return controller2.button(1);
  }

  public Trigger lollipopLeftTrigger() {
    return controller2.button(5);
  }

  public Trigger lollipopCenterTrigger() {
    return controller2.button(6);
  }

  public Trigger lollipopRightTrigger() {
    return controller2.button(7);
  }
}
