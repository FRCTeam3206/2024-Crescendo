package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class CycleLEDCommand extends Command {
  private final LEDSubsystem leds;
  private final double delaySeconds;
  private Timer timer = new Timer();

  public CycleLEDCommand(LEDSubsystem leds, double delaySeconds) {
    this.leds = leds;
    this.delaySeconds = delaySeconds;
    addRequirements(leds);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (timer.advanceIfElapsed(this.delaySeconds)) {
      this.leds.rotate();
    }
  }
}
