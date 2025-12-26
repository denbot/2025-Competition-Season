package frc.robot.subsystems.led;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Arrays;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import static edu.wpi.first.units.Units.Milliseconds;

/**
 * Subsystem for controlling Addressable LEDs on the robot.
 *
 * <p>This controller manages an {@link AddressableLED} strip by mapping physical pixels to
 * {@link LEDSubsystem} virtual resources. This allows multiple commands to run on different
 * sections of the strip simultaneously without manual conflict checking.
 */
public class LEDController {

  /*
   * These are specific buffers that can be addressed as a unit instead of always specifying start/end. They also have
   * the advantage that the range is validated in the subsystem constructor as no one else can make a buffer view as
   * long as the AddressableLEDBuffer itself is private. Keep them private if you don't want to directly allow another
   * class to use them.
   *
   * Technically, we could expose the AddressableLEDBuffer and let others use it directly or create their own buffer
   * view to use. I chose to go with more encapsulation rather than less in this case. If you did want to allow the user
   * to make their own views, I would expose the createView method so we can guarantee the stripToSubsystems map has
   * been updated with the correct values. We would do it on the fly, but there's no way to determine what the physical
   * LED indexes are based on the logical indexes used in the view.
   */
  /**
   * Buffer view for the left section of the LED strip.
   */
  public final AddressableLEDBufferView leftBuffer;
  /**
   * Buffer view for the center section of the LED strip.
   */
  public final AddressableLEDBufferView centerBuffer;
  /**
   * Buffer view for the right section of the LED strip.
   */
  public final AddressableLEDBufferView rightBuffer;

  private final AddressableLEDBufferView l2LeftBuffer;
  private final AddressableLEDBufferView l2RightBuffer;
  private final AddressableLEDBufferView l3LeftBuffer;
  private final AddressableLEDBufferView l3RightBuffer;
  private final AddressableLEDBufferView l4LeftBuffer;
  private final AddressableLEDBufferView l4RightBuffer;

  // Internal state, used mostly in the periodic function or when we need to refer to the full LED string internally
  private final AddressableLED LEDString;
  private final AddressableLEDBuffer baseLEDBuffer;

  private final Map<Integer, LEDSubsystem> ledSubsystemMap;
  private final IdentityHashMap<Object, LEDSubsystem[]> stripToSubsystems;

  /**
   * Creates a new LEDSubsystem.
   *
   * @param numberOfLEDs The total number of LEDs in the strip.
   */
  public LEDController(int numberOfLEDs) {
    ledSubsystemMap = LEDSubsystem.getLEDMap(numberOfLEDs);
    stripToSubsystems = new IdentityHashMap<>();

    LEDString = new AddressableLED(0);
    baseLEDBuffer = new AddressableLEDBuffer(numberOfLEDs);

    // Let's go ahead and put the cache for our full LED strip in
    stripToSubsystems.put(
        baseLEDBuffer,
        ledSubsystemMap.values().toArray(new LEDSubsystem[0])
    );

    LEDString.setLength(numberOfLEDs);
    LEDString.start();

    // Let's set up views for the individual sections we care about
    l2LeftBuffer = createView(0, 3);
    l2RightBuffer = createView(17, 20);

    l3LeftBuffer = createView(3, 6);
    l3RightBuffer = createView(14, 17);

    l4LeftBuffer = createView(6, 9);
    l4RightBuffer = createView(11, 14);

    leftBuffer = createView(0, 6);
    centerBuffer = createView(7, 13);
    rightBuffer = createView(14, 20);

    // Kick off our LED updates every event loop. This allows the LED controller to update the LED string while our
    // actual LEDSubsystem enum helps ensure no single LED is running more than one command.
    Commands.run(() -> LEDString.setData(baseLEDBuffer))
        .ignoringDisable(true)
        .withName("LED Data Updater")
        .schedule();
  }

  private AddressableLEDBufferView createView(int startIndex, int endIndex) {
    AddressableLEDBufferView view = baseLEDBuffer.createView(startIndex, endIndex);

    LEDSubsystem[] subsystems = IntStream
        .rangeClosed(startIndex, endIndex)
        .mapToObj(ledSubsystemMap::get)
        .toArray(LEDSubsystem[]::new);

    stripToSubsystems.put(view, subsystems);

    return view;
  }

  /**
   * Runs a specific {@link LEDPattern} on a given LED buffer or view.
   *
   * @param ledStrip The LED buffer or view to apply this pattern to.
   * @param pattern  The specific pattern to apply.
   * @param <LED>    The type of the LED buffer or view (must implement {@link LEDReader} and {@link LEDWriter}).
   * @return A command that applies the pattern to the LED strip continuously.
   */
  public <LED extends LEDReader & LEDWriter> Command run(LED ledStrip, LEDPattern pattern) {
    return Commands.run(() -> pattern.applyTo(ledStrip), stripToSubsystems.get(ledStrip))
        .withName("LED Control")
        .ignoringDisable(true);
  }

  /**
   * Runs a specific {@link LEDPattern} on the entire LED strip.
   *
   * @param pattern The pattern to apply.
   * @return A command that applies the pattern to the entire strip continuously.
   */
  public Command run(LEDPattern pattern) {
    return run(baseLEDBuffer, pattern);
  }

  /**
   * Fills a given LED buffer or view with a solid color.
   *
   * @param ledStrip The LED buffer or view to fill.
   * @param color    The color to fill with.
   * @param <LED>    The type of the LED buffer or view.
   * @return A command that fills the LED strip with the color.
   */
  public <LED extends LEDReader & LEDWriter> Command fill(LED ledStrip, Color color) {
    return run(ledStrip, LEDPattern.solid(color));
  }

  /**
   * Fills the entire LED strip with a solid color.
   *
   * @param color The color to fill with.
   * @return A command that fills the entire strip with the color.
   */
  public Command fill(Color color) {
    return fill(baseLEDBuffer, color);
  }

  /**
   * Temporarily runs a pattern on a specific LED section for a given duration,
   * then clears that section to black.
   *
   * @param ledStrip The LED buffer or view to apply the pattern to.
   * @param pattern  The pattern to apply temporarily.
   * @param timeOn   The duration to run the pattern.
   * @param <LED>    The type of the LED buffer or view.
   * @return A command that runs the pattern temporarily.
   */
  public <LED extends LEDReader & LEDWriter> Command temporary(
      LED ledStrip,
      LEDPattern pattern,
      Time timeOn
  ) {
    return run(ledStrip, pattern)
        .withTimeout(timeOn)
        .andThen(fill(ledStrip, Color.kBlack));
  }

  /**
   * Temporarily fills a specific LED section with a solid color for a given duration,
   * then clears that section to black.
   *
   * @param ledStrip The LED buffer or view to fill.
   * @param color    The color to fill with temporarily.
   * @param timeOn   The duration to keep the color.
   * @param <LED>    The type of the LED buffer or view.
   * @return A command that fills the section temporarily.
   */
  public <LED extends LEDReader & LEDWriter> Command temporary(
      LED ledStrip,
      Color color,
      Time timeOn
  ) {
    return temporary(ledStrip, LEDPattern.solid(color), timeOn);
  }

  /**
   * Temporarily runs a pattern on the entire LED strip for a given duration,
   * then clears the strip to black.
   *
   * @param pattern The pattern to apply temporarily.
   * @param timeOn  The duration to run the pattern.
   * @return A command that runs the pattern temporarily.
   */
  public Command temporary(
      LEDPattern pattern,
      Time timeOn
  ) {
    return temporary(baseLEDBuffer, pattern, timeOn);
  }

  /**
   * Temporarily fills the entire LED strip with a solid color for a given duration,
   * then clears the strip to black.
   *
   * @param color  The color to fill with temporarily.
   * @param timeOn The duration to keep the color.
   * @return A command that fills the strip temporarily.
   */
  public Command temporary(
      Color color,
      Time timeOn
  ) {
    return temporary(baseLEDBuffer, LEDPattern.solid(color), timeOn);
  }

  /**
   * Indicates Level 2 status by blinking yellow on the L2 sections.
   *
   * @return A command that performs the L2 indication.
   */
  public Command indicateL2() {
    return indicateLevel(l2LeftBuffer, l2RightBuffer, Color.kYellow);
  }

  /**
   * Indicates Level 3 status by blinking orange on the L3 sections.
   *
   * @return A command that performs the L3 indication.
   */
  public Command indicateL3() {
    return indicateLevel(l3LeftBuffer, l3RightBuffer, Color.kOrange);
  }

  /**
   * Indicates Level 4 status by blinking red on the L4 sections.
   *
   * @return A command that performs the L4 indication.
   */
  public Command indicateL4() {
    return indicateLevel(l4LeftBuffer, l4RightBuffer, Color.kRed);
  }

  /**
   * Helper method to blink a color on two specific sections for a set duration.
   *
   * @param left  The left buffer view.
   * @param right The right buffer view.
   * @param color The color to blink.
   * @return A command that blinks the color and then clears to black.
   */
  private Command indicateLevel(
      AddressableLEDBufferView left,
      AddressableLEDBufferView right,
      Color color
  ) {
    LEDPattern pattern = LEDPattern
        .solid(color)
        .blink(Milliseconds.of(250));

    LEDSubsystem[] subsystems = Stream.concat(
            Arrays.stream(stripToSubsystems.get(left)),
            Arrays.stream(stripToSubsystems.get(right))
        )
        .toArray(LEDSubsystem[]::new);

    return Commands.run(() -> {
              pattern.applyTo(left);
              pattern.applyTo(right);
            },
            subsystems
        )
        .ignoringDisable(true)
        .withName("LED Scoring Level")
        .withTimeout(Milliseconds.of(500))
        .andThen(fill(Color.kBlack));
  }

  /**
   * Runs a rainbow pattern on a specific LED section.
   *
   * @param ledStrip The LED buffer or view to apply the rainbow to.
   * @param <LED>    The type of the LED buffer or view.
   * @return A command that runs the rainbow pattern.
   */
  public <LED extends LEDReader & LEDWriter> Command rainbow(LED ledStrip) {
    LEDPattern rainbowPattern = LEDPattern.rainbow(255, 128);

    return run(ledStrip, rainbowPattern);
  }

  /**
   * Runs a rainbow pattern on the entire LED strip.
   *
   * @return A command that runs the rainbow pattern on the entire strip.
   */
  public Command rainbow() {
    return rainbow(baseLEDBuffer);
  }
}
