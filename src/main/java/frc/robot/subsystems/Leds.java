package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
  private AddressableLED ledString;
  private AddressableLEDBuffer ledBuffer;
  private int amountOfLights;
  private int m_rainbowFirstPixelHue = 0;

  public Leds() {
    ledString = new AddressableLED(0);
    this.amountOfLights = 21;
    ledBuffer = new AddressableLEDBuffer(this.amountOfLights);

    ledString.setLength(this.amountOfLights);
    this.update();
    ledString.start();
  }

  public void update() {
    ledString.setData(ledBuffer);
  }

  public Command indicateL2() {
    return Commands.run(() -> flashSection(0, 3, 150, 255, 255, 0.25))
        .alongWith(Commands.run(() -> flashSection(18, 21, 150, 255, 255, 0.25)))
        .withTimeout(0.5)
        .andThen(Commands.runOnce(() -> fullSolid(0, 0, 0)));
  }

  public Command indicateL3() {
    return Commands.run(() -> flashSection(3, 6, 150, 255, 255, 0.25))
        .alongWith(Commands.run(() -> flashSection(15, 18, 150, 255, 255, 0.25)))
        .withTimeout(0.5)
        .andThen(Commands.runOnce(() -> fullSolid(0, 0, 0)));
  }

  public Command indicateL4() {
    return Commands.run(() -> flashSection(6, 9, 150, 255, 255, 0.25))
        .alongWith(Commands.run(() -> flashSection(12, 15, 150, 255, 255, 0.25)))
        .withTimeout(0.5)
        .andThen(Commands.runOnce(() -> fullSolid(0, 0, 0)));
  }

  public void solidInSection(int start, int finish, int hue, int sat, int val) {
    for (var i = start; i < finish; i++) {
      ledBuffer.setHSV(i, hue, sat, val);
    }
    update();
  }

  public void fullSolid(int hue, int sat, int val) {
    this.solidInSection(0, amountOfLights, hue, sat, val);
  }

  public void flash(int hue, int sat, int val, double flashRate) {
    if (Timer.getFPGATimestamp() % flashRate < flashRate / 2) {
      for (var i = 0; i < 21; i++) {
        ledBuffer.setHSV(i, hue, sat, val);
      }
    } else {
      for (var i = 0; i < 21; i++) {
        ledBuffer.setHSV(i, hue, 0, 255);
      }
    }

    update();
  }

  public void flashSection(int start, int end, int hue, int sat, int val, double flashRate) {
    if (Timer.getFPGATimestamp() % flashRate < flashRate / 2) {
      for (var i = start; i < end; i++) {
        ledBuffer.setHSV(i, hue, sat, val);
      }
    } else {
      for (var i = 0; i < 21; i++) {
        ledBuffer.setHSV(i, hue, 0, 0);
      }
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
