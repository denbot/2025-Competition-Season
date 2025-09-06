package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
  private AddressableLED ledString;
  private AddressableLEDBuffer ledBuffer;
  private int amountOfLights;
  private int m_rainbowFirstPixelHue = 0;

  public Leds() {
    ledString = new AddressableLED(0);
    this.amountOfLights = 24;
    ledBuffer = new AddressableLEDBuffer(this.amountOfLights);

    ledString.setLength(this.amountOfLights);
    this.update();
    ledString.start();
  }

  public void update() {
    ledString.setData(ledBuffer);
  }

  public void solid(int start, int finish, int hue, int sat, int val) {
    for (var i = start; i < finish; i++) {
      ledBuffer.setHSV(i, hue, sat, val);
    }
    update();
  }

  public void rainbow() {

    // For every pixel

    for (var i = 0; i < ledBuffer.getLength(); i++) {

      // Calculate the hue - hue is easier for rainbows because the color

      // shape is a circle so only one value needs to precess

      final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;

      // Set the value

      ledBuffer.setHSV(i, hue, 255, 128);
    }

    // Increase by to make the rainbow "move"

    m_rainbowFirstPixelHue += 3;

    // Check bounds

    m_rainbowFirstPixelHue %= 180;
    update();
  }
}
