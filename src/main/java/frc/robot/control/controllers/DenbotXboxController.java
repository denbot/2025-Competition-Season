package frc.robot.control.controllers;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A custom controller wrapper just to give some extra helper methods that aren't exposed in the base class.
 */
public class DenbotXboxController extends CommandXboxController {
  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public DenbotXboxController(int port) {
    super(port);
  }

  /**
   * Constructs a Trigger instance based around the 0-degree angle (up) of the default (index 0)
   * POV on the HID.
   *
   * @param eventLoop The event loop to attach this trigger to.
   * @return a Trigger instance based around the 0-degree angle of a POV on the HID.
   */
  public Trigger povUp(EventLoop eventLoop) {
    return pov(0, 0, eventLoop);
  }

  /**
   * Constructs a Trigger instance based around the 45-degree angle (right up) of the default (index
   * 0) POV on the HID.
   *
   * @param eventLoop The event loop to attach this trigger to.
   * @return a Trigger instance based around the 45-degree angle of a POV on the HID.
   */
  public Trigger povUpRight(EventLoop eventLoop) {
    return pov(0, 45, eventLoop);
  }

  /**
   * Constructs a Trigger instance based around the 90-degree angle (right) of the default (index 0)
   * POV on the HID.
   *
   * @param eventLoop The event loop to attach this trigger to.
   * @return a Trigger instance based around the 90-degree angle of a POV on the HID.
   */
  public Trigger povRight(EventLoop eventLoop) {
    return pov(0, 90, eventLoop);
  }

  /**
   * Constructs a Trigger instance based around the 135-degree angle (right down) of the default
   * (index 0) POV on the HID.
   *
   * @param eventLoop The event loop to attach this trigger to.
   * @return a Trigger instance based around the 135-degree angle of a POV on the HID.
   */
  public Trigger povDownRight(EventLoop eventLoop) {
    return pov(0, 135, eventLoop);
  }

  /**
   * Constructs a Trigger instance based around the 180-degree angle (down) of the default (index 0)
   * POV on the HID.
   *
   * @param eventLoop The event loop to attach this trigger to.
   * @return a Trigger instance based around the 180-degree angle of a POV on the HID.
   */
  public Trigger povDown(EventLoop eventLoop) {
    return pov(0, 180, eventLoop);
  }

  /**
   * Constructs a Trigger instance based around the 225-degree angle (down left) of the default
   * (index 0) POV on the HID.
   *
   * @param eventLoop The event loop to attach this trigger to.
   * @return a Trigger instance based around the 225-degree angle of a POV on the HID.
   */
  public Trigger povDownLeft(EventLoop eventLoop) {
    return pov(0, 225, eventLoop);
  }

  /**
   * Constructs a Trigger instance based around the 270-degree angle (left) of the default (index 0)
   * POV on the HID.
   *
   * @param eventLoop The event loop to attach this trigger to.
   * @return a Trigger instance based around the 270-degree angle of a POV on the HID.
   */
  public Trigger povLeft(EventLoop eventLoop) {
    return pov(0, 270, eventLoop);
  }

  /**
   * Constructs a Trigger instance based around the 315-degree angle (left up) of the default (index
   * 0) POV on the HID.
   *
   * @param eventLoop The event loop to attach this trigger to.
   * @return a Trigger instance based around the 315-degree angle of a POV on the HID.
   */
  public Trigger povUpLeft(EventLoop eventLoop) {
    return pov(0, 315, eventLoop);
  }

  /**
   * Constructs a Trigger instance based around the center (not pressed) position of the default
   * (index 0) POV on the HID.
   *
   * @param eventLoop The event loop to attach this trigger to.
   * @return a Trigger instance based around the center position of a POV on the HID.
   */
  public Trigger povCenter(EventLoop eventLoop) {
    return pov(0, -1, eventLoop);
  }

  /**
   * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
   * will be true when the axis value is greater than 0.5.
   *
   * @param eventLoop The event loop to attach this trigger to.
   * @return a Trigger instance that is true when the left trigger's axis exceeds 0.5.
   */
  public Trigger leftTrigger(EventLoop eventLoop) {
    return leftTrigger(0.5, eventLoop);
  }

  /**
   * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
   * will be true when the axis value is greater than 0.5.
   *
   * @param eventLoop The event loop to attach this trigger to.
   * @return a Trigger instance that is true when the right trigger's axis exceeds 0.5.
   */
  public Trigger rightTrigger(EventLoop eventLoop) {
    return rightTrigger(0.5, eventLoop);
  }
}
