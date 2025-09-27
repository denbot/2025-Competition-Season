package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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
    return getFlashCommand(0, 3, 60, 255, 255, 0.25, 0.5)
        .alongWith(getFlashCommand(18, 21, 60, 255, 255, 0.25, 0.5));
  }

  public Command indicateL3() {
    return getFlashCommand(3, 6, 30, 255, 255, 0.25, 0.5)
        .alongWith(getFlashCommand(15, 18, 30, 255, 255, 0.25, 0.5));
  }

  public Command indicateL4() {
    return getFlashCommand(6, 9, 0, 255, 255, 0.25, 0.5)
        .alongWith(getFlashCommand(12, 15, 0, 255, 255, 0.25, 0.5));
  }

  public void solidInSection(int start, int finish, int hue, int sat, int val) {
    for (var i = start; i < finish; i++) {
      ledBuffer.setHSV(i, hue, sat, val);
    }
    update();
  }

  public void gradientInSection(int start, int finish, int hue, int sat, int val) {
    if (start < finish) {
      for (var i = start; i < finish; i++) {
        ledBuffer.setHSV(i, hue, sat, val - ((i / finish) * val));
      }
    } else {
      for (var i = start; i > finish; i--) {
        ledBuffer.setHSV(i, hue, sat, val * (i / start));
      }
    }
    update();
  }

  public Command getFlashCommand(
      int start, int finish, int hue, int sat, int val, double flashRate, double duration) {
    return Commands.repeatingSequence(
            Commands.runOnce(() -> solidInSection(start, finish, hue, sat, val)),
            new WaitCommand(flashRate),
            Commands.runOnce(() -> solidInSection(start, finish, 0, 0, 0)),
            new WaitCommand(flashRate))
        .withTimeout(duration)
        .andThen(() -> solidInSection(start, finish, 0, 0, 0));
  }

  public void solidInSectionLeft(int hue, int sat, int val) {
    this.solidInSection(0, 7, hue, sat, val);
  }

  public void solidInSectionCenter(int hue, int sat, int val) {
    this.solidInSection(7, 14, hue, sat, val / 2);
  }

  public void solidInSectionRight(int hue, int sat, int val) {
    this.solidInSection(14, 21, hue, sat, val);
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
