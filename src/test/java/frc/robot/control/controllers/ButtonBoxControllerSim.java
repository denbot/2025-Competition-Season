package frc.robot.control.controllers;

import edu.wpi.first.wpilibj.simulation.GenericHIDSim;
import frc.robot.Constants;

public class ButtonBoxControllerSim {
  private final GenericHIDSim controller1Sim = new GenericHIDSim(Constants.OperatorConstants.kButtonBoxControllerAPort);
  private final GenericHIDSim controller2Sim = new GenericHIDSim(Constants.OperatorConstants.kButtonBoxControllerBPort);

  public ButtonBoxControllerSim() {
    // Setting the button count is required for the sim to work, otherwise getting button values will always return false
    controller1Sim.setButtonCount(12);
    controller2Sim.setButtonCount(12);
  }

  public void setL4Trigger(boolean value) {
    controller1Sim.setRawButton(4, value);
  }

  public void setL3Trigger(boolean value) {
    controller1Sim.setRawButton(5, value);
  }

  public void setL2Trigger(boolean value) {
    controller1Sim.setRawButton(6, value);
  }

  public void setL1Trigger(boolean value) {
    controller1Sim.setRawButton(7, value);
  }

  public void setSpearTrigger(boolean value) {
    controller2Sim.setRawButton(4, value);
  }

  public void setTwoLeftTrigger(boolean value) {
    controller1Sim.setRawButton(2, value);
  }

  public void setTwoRightTrigger(boolean value) {
    controller1Sim.setRawButton(3, value);
  }

  public void setFourLeftTrigger(boolean value) {
    controller1Sim.setRawButton(11, value);
  }

  public void setFourRightTrigger(boolean value) {
    controller1Sim.setRawButton(8, value);
  }

  public void setSixLeftTrigger(boolean value) {
    controller2Sim.setRawButton(12, value);
  }

  public void setSixRightTrigger(boolean value) {
    controller1Sim.setRawButton(12, value);
  }

  public void setEightLeftTrigger(boolean value) {
    controller2Sim.setRawButton(8, value);
  }

  public void setEightRightTrigger(boolean value) {
    controller2Sim.setRawButton(11, value);
  }

  public void setTenLeftTrigger(boolean value) {
    controller2Sim.setRawButton(3, value);
  }

  public void setTenRightTrigger(boolean value) {
    controller2Sim.setRawButton(2, value);
  }

  public void setTwelveLeftTrigger(boolean value) {
    controller1Sim.setRawButton(1, value);
  }

  public void setTwelveRightTrigger(boolean value) {
    controller2Sim.setRawButton(1, value);
  }

  public void setLollipopLeftTrigger(boolean value) {
    controller2Sim.setRawButton(5, value);
  }

  public void setLollipopCenterTrigger(boolean value) {
    controller2Sim.setRawButton(6, value);
  }

  public void setLollipopRightTrigger(boolean value) {
    controller2Sim.setRawButton(7, value);
  }

  public void notifyNewData() {
    controller1Sim.notifyNewData();
    controller2Sim.notifyNewData();
  }
}
