package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue = 0;
  private int count = 0;
  private int count2 = 0;
  /*
   * showColor 0 - Rainbow
   * showColor 1 - Solid color (blue)
   * showColor 2 - slow theatre chase style alternation between r1,g1,b1 &
   * r2,g2,b2
   * showColor 3 - rainbow for 1 sec, theatre chase for 1 sec
   */
  public String showColor = "blue";

  public Lights() {
    // PWM port 0
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(1);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(150);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void periodic() {
    // Chooses and uses a mode for the lights based on showColor
    if (showColor == "rainbow") {
      rainbow(m_rainbowFirstPixelHue);
      m_led.setData(m_ledBuffer);
      m_rainbowFirstPixelHue += 3;
      // Check bounds
      m_rainbowFirstPixelHue %= 180;
    } else if (showColor == "blue" || showColor == "alternate") {
      if (count >= 25) {
        ColorSwitch(0, 0, 200, 150, 150, 150);
        m_led.setData(m_ledBuffer);
        count = 0;
      }
      count++;
    } else if (showColor == "fancy") {
      if (count2 < 50) {
        rainbow(m_rainbowFirstPixelHue);
        m_led.setData(m_ledBuffer);
      } else if (count2 < 100) {
        if (count == 13) {
          ColorSwitch(0, 0, 200, 150, 150, 150);
          m_led.setData(m_ledBuffer);
          count = 0;
        }
        count++;
      } else if (count2 > 100) {
        count2 = 0;
      }
      count2++;
    }
    m_led.setData(m_ledBuffer);
  }

  public void changeColor(String color) {
    showColor = color;
  }

  /** sets all LED's to the rgb color specified from the three aproprately named variables */
  public void setLightColor(int ColorRed, int ColorGreen, int ColorBlue) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, ColorRed, ColorGreen, ColorBlue);
    }
    showColor = "blue";
    m_led.setData(m_ledBuffer);
  }

  /**
   * makes a fun rainbow pattern
   *
   * <p>Copied from the addressable LED library WPILib documentation
   */
  private void rainbow(int m_rainbowFirstPixelHue) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
  }

  public void stop() {
    m_led.stop();
  }

  /** Alternates colors for each led, then switches them every time it's called */
  private void ColorSwitch(int r1, int g1, int b1, int r2, int g2, int b2) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i%2 == 0) {
        m_ledBuffer.setRGB(i, r1, g1, b1);
        SmartDashboard.putString("color", "poop");
      } else {
        m_ledBuffer.setRGB(i, r2, g2, b2);
        SmartDashboard.putString("color", "pee");
      }
    }
  }
}
