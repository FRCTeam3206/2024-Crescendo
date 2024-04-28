package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CycleLEDCommand;
import monologue.Annotations.Log;
import monologue.Logged;

public class LEDSubsystem extends SubsystemBase implements Logged {

  private AddressableLED leds;
  @Log private AddressableLEDBuffer ledBuffer;

  @SuppressWarnings("unused")
  private AddressableLEDSim ledsSim;

  public LEDSubsystem(int port, int length) {
    leds = new AddressableLED(port);
    ledsSim = new AddressableLEDSim(leds);
    ledBuffer = new AddressableLEDBuffer(length);

    leds.setLength(ledBuffer.getLength());
    leds.setData(ledBuffer);
    leds.start();
  }

  private void initRainbow(int cycles) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      final int hue = (i * 180 * cycles / ledBuffer.getLength()) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
      leds.setData(ledBuffer);
    }
  }

  public void rotate() {
    Color last = ledBuffer.getLED(0);
    for (int i = 0; i < ledBuffer.getLength() - 1; i++) {
      ledBuffer.setLED(i, ledBuffer.getLED(i + 1));
    }
    ledBuffer.setLED(ledBuffer.getLength() - 1, last);
    leds.setData(ledBuffer);
  }

  public Command cycleCommand(double delaySeconds) {
    return new CycleLEDCommand(this, delaySeconds).ignoringDisable(true);
  }

  public Command solidColorCommand(Color8Bit color) {
    return runOnce(
            () -> {
              for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, color);
              }
              leds.setData(ledBuffer);
            })
        .andThen(run(() -> {}))
        .ignoringDisable(true);
  }

  public Command rainbowCommand(double delaySeconds, int cycles) {
    return runOnce(() -> initRainbow(cycles))
        .ignoringDisable(true)
        .andThen(cycleCommand(delaySeconds));
  }

  public Command rainbowCommand(double delaySeconds) {
    return rainbowCommand(delaySeconds, 1);
  }
}
