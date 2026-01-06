package frc.robot.control.controllers;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.Map;
import java.util.function.Consumer;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Validates the mapping between {@link ButtonBoxControllerSim} and {@link ButtonBoxController}.
 * <p>
 * While both classes are internal to our project, this test is critical because they are
 * manually synchronized. If a button index is changed in the controller but not updated
 * in the simulator (or vice versa), simulation-based testing and driver practice will
 * give false results. This test ensures that "setL4Trigger" in the sim actually
 * triggers the "L4Trigger" in the code.
 */
public class ButtonBoxControllerSimTest {
  @BeforeEach
  public void setup() {
    // You must initialize the HAL before running tests that interacts with controllers
    assertTrue(HAL.initialize(500, 0));
  }

  /**
   * We need to test that each button correctly matches in the simulation code with the button in the real controller.
   */
  @Test
  void verifyButtonMapping() {
    var buttonBoxController = new ButtonBoxController();
    var buttonBoxControllerSim = new ButtonBoxControllerSim();

    Map<Consumer<Boolean>, Trigger> simToTriggerMap = Map.ofEntries(
        Map.entry(buttonBoxControllerSim::setL1Trigger, buttonBoxController.L1Trigger()),
        Map.entry(buttonBoxControllerSim::setL2Trigger, buttonBoxController.L2Trigger()),
        Map.entry(buttonBoxControllerSim::setL3Trigger, buttonBoxController.L3Trigger()),
        Map.entry(buttonBoxControllerSim::setL4Trigger, buttonBoxController.L4Trigger()),
        Map.entry(buttonBoxControllerSim::setTwoLeftTrigger, buttonBoxController.twoLeftTrigger()),
        Map.entry(buttonBoxControllerSim::setTwoRightTrigger, buttonBoxController.twoRightTrigger()),
        Map.entry(buttonBoxControllerSim::setFourLeftTrigger, buttonBoxController.fourLeftTrigger()),
        Map.entry(buttonBoxControllerSim::setFourRightTrigger, buttonBoxController.fourRightTrigger()),
        Map.entry(buttonBoxControllerSim::setSixLeftTrigger, buttonBoxController.sixLeftTrigger()),
        Map.entry(buttonBoxControllerSim::setSixRightTrigger, buttonBoxController.sixRightTrigger()),
        Map.entry(buttonBoxControllerSim::setEightLeftTrigger, buttonBoxController.eightLeftTrigger()),
        Map.entry(buttonBoxControllerSim::setEightRightTrigger, buttonBoxController.eightRightTrigger()),
        Map.entry(buttonBoxControllerSim::setTenLeftTrigger, buttonBoxController.tenLeftTrigger()),
        Map.entry(buttonBoxControllerSim::setTenRightTrigger, buttonBoxController.tenRightTrigger()),
        Map.entry(buttonBoxControllerSim::setTwelveLeftTrigger, buttonBoxController.twelveLeftTrigger()),
        Map.entry(buttonBoxControllerSim::setTwelveRightTrigger, buttonBoxController.twelveRightTrigger()),
        Map.entry(buttonBoxControllerSim::setSpearTrigger, buttonBoxController.clearTrigger()),
        Map.entry(buttonBoxControllerSim::setLollipopUpTrigger, buttonBoxController.lollipopUpTrigger()),
        Map.entry(buttonBoxControllerSim::setLollipopCenterTrigger, buttonBoxController.lollipopCenterTrigger()),
        Map.entry(buttonBoxControllerSim::setLollipopDownTrigger, buttonBoxController.lollipopDownTrigger())
    );

    /*
     For each button under test, we want to enable it and disable all the others. At that point only the trigger under
     test should be enabled.
    */
    for (var entryUnderTest : simToTriggerMap.entrySet()) {
      var simButtonUnderTest = entryUnderTest.getKey();
      var actualTrigger = entryUnderTest.getValue();

      // Set all buttons to false except the one we're testing
      for (Consumer<Boolean> simButton : simToTriggerMap.keySet()) {
        simButton.accept(simButton == simButtonUnderTest);
      }

      buttonBoxControllerSim.notifyNewData();
      CommandScheduler.getInstance().run();

      // Now test that they're all false except our actual trigger
      for (Trigger trigger : simToTriggerMap.values()) {
        assertEquals(trigger == actualTrigger, trigger.getAsBoolean());
      }
    }
  }
}
