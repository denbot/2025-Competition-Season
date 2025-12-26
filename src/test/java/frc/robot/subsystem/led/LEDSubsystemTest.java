package frc.robot.subsystem.led;

import frc.robot.subsystems.led.LEDSubsystem;
import org.junit.jupiter.api.Test;

import java.util.HashSet;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.*;

public class LEDSubsystemTest {
  /**
   * The LEDSubsystem enum values all take the form of LED_[number]. This test fails if they don't or if they're not a
   * counted sequence (e.g., if there are any skips, or they don't start at 0, etc.).
   */
  @Test
  void allEnumValuesAreNamedProperly() {
    Set<Integer> accountedForNumbers = new HashSet<>();

    for (LEDSubsystem instance : LEDSubsystem.values()) {
      String name = instance.name();
      String failMessage = name + " is not in the expected format of LED_[num]";

      assertTrue(name.startsWith("LED_"), failMessage);

      try {
        int led_number = Integer.parseInt(name.substring(4));

        if (accountedForNumbers.contains(led_number)) {
          fail("Duplicate Enum value. This can happen for values such as LED_X and LED_0X");
        }

        accountedForNumbers.add(led_number);
      } catch (NumberFormatException e) {
        fail(failMessage);
      }
    }
  }

  @Test
  void canGetLEDMap() {
    var map = LEDSubsystem.getLEDMap(3);

    assertEquals(3, map.size());

    assertTrue(map.containsKey(0));
    assertEquals(LEDSubsystem.LED_0, map.get(0));
    assertTrue(map.containsKey(1));
    assertEquals(LEDSubsystem.LED_1, map.get(1));
    assertTrue(map.containsKey(2));
    assertEquals(LEDSubsystem.LED_2, map.get(2));
  }

  @Test
  void getLEDMapFailsIfTooManyLEDsAreRequested() {
    // I know you're thinking about making a generator to generate this many LEDSubsystem instances.
    // I can't stop you, good luck.

    assertThrows(LEDSubsystem.MoreLEDsRequestedThanWeHave.class, () -> {
      LEDSubsystem.getLEDMap(Integer.MAX_VALUE);
    });
  }
}
