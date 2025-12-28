package frc.robot.control.controllers;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBoxController {
  private static final CommandGenericHID controller1 = new CommandGenericHID(1);
  private static final CommandGenericHID controller2 = new CommandGenericHID(2);

  public boolean isControllerOneConnected() {
    return controller1.isConnected();
  }

  public boolean isControllerTwoConnected() {
    return controller2.isConnected();
  }

  public Trigger L4Trigger() {
    return L4Trigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger L4Trigger(EventLoop loop) {
    return controller1.button(4, loop);
  }

  public Trigger L3Trigger() {
    return L3Trigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger L3Trigger(EventLoop loop) {
    return controller1.button(5, loop);
  }

  public Trigger L2Trigger() {
    return L2Trigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger L2Trigger(EventLoop loop) {
    return controller1.button(6, loop);
  }

  public Trigger L1Trigger() {
    return L1Trigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger L1Trigger(EventLoop loop) {
    return controller1.button(7, loop);
  }

  public Trigger spearTrigger() {
    return spearTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger spearTrigger(EventLoop loop) {
    return controller2.button(4, loop);
  }

  public Trigger twoLeftTrigger() {
    return twoLeftTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger twoLeftTrigger(EventLoop loop) {
    return controller1.button(2, loop);
  }

  public Trigger twoRightTrigger() {
    return twoRightTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger twoRightTrigger(EventLoop loop) {
    return controller1.button(3, loop);
  }

  public Trigger fourLeftTrigger() {
    return fourLeftTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger fourLeftTrigger(EventLoop loop) {
    return controller1.button(11, loop);
  }

  public Trigger fourRightTrigger() {
    return fourRightTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger fourRightTrigger(EventLoop loop) {
    return controller1.button(8, loop);
  }

  public Trigger sixLeftTrigger() {
    return sixLeftTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger sixLeftTrigger(EventLoop loop) {
    return controller2.button(12, loop);
  }

  public Trigger sixRightTrigger() {
    return sixRightTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger sixRightTrigger(EventLoop loop) {
    return controller1.button(12, loop);
  }

  public Trigger eightLeftTrigger() {
    return eightLeftTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger eightLeftTrigger(EventLoop loop) {
    return controller2.button(8, loop);
  }

  public Trigger eightRightTrigger() {
    return eightRightTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger eightRightTrigger(EventLoop loop) {
    return controller2.button(11, loop);
  }

  public Trigger tenLeftTrigger() {
    return tenLeftTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger tenLeftTrigger(EventLoop loop) {
    return controller2.button(3, loop);
  }

  public Trigger tenRightTrigger() {
    return tenRightTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger tenRightTrigger(EventLoop loop) {
    return controller2.button(2, loop);
  }

  public Trigger twelveLeftTrigger() {
    return twelveLeftTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger twelveLeftTrigger(EventLoop loop) {
    return controller1.button(1, loop);
  }

  public Trigger twelveRightTrigger() {
    return twelveRightTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger twelveRightTrigger(EventLoop loop) {
    return controller2.button(1, loop);
  }

  public Trigger lollipopLeftTrigger() {
    return lollipopLeftTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger lollipopLeftTrigger(EventLoop loop) {
    return controller2.button(5, loop);
  }

  public Trigger lollipopCenterTrigger() {
    return lollipopCenterTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger lollipopCenterTrigger(EventLoop loop) {
    return controller2.button(6, loop);
  }

  public Trigger lollipopRightTrigger() {
    return lollipopRightTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger lollipopRightTrigger(EventLoop loop) {
    return controller2.button(7, loop);
  }
}
