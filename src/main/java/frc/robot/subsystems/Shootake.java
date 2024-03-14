package frc.robot.subsystems;

import static frc.robot.Constants.ShootakeConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OutreachConstants;
import frc.robot.Constants.ShootakeConstants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import monologue.Annotations.Log;
import monologue.Logged;

public class Shootake extends SubsystemBase implements Logged {
  CANSparkMax topRoller = new CANSparkMax(kTopCANID, MotorType.kBrushless);
  CANSparkMax bottomRoller = new CANSparkMax(kBottomCANID, MotorType.kBrushless);
  Servo finger = new Servo(kFingerPort);
  DigitalInput hasNoteSensor = new DigitalInput(ShootakeConstants.kNoteSensorChannel);
  Debouncer shootDebounce = new Debouncer(.125);

  public Shootake() {
    topRoller.setSmartCurrentLimit(23);
    bottomRoller.setSmartCurrentLimit(23);
  }

  @Log
  private double getAverageSpeed() {
    return -(topRoller.getEncoder().getVelocity() + bottomRoller.getEncoder().getVelocity()) / 2;
  }

  @Log
  private double getTopRollerSpeed() {
    return -topRoller.getEncoder().getVelocity();
  }

  @Log
  private double getBottomRollerSpeed() {
    return -bottomRoller.getEncoder().getVelocity();
  }

  public void setSpeed(double speed) {
    topRoller.set(speed * OutreachConstants.kMaxShootPercentSpeed);
    bottomRoller.set(speed * OutreachConstants.kMaxShootPercentSpeed);
  }

  public Command setSpeedCommand(Supplier<Double> speed) {
    return new SequentialCommandGroup(
      this.run(() -> setSpeed(speed.get()))
    );
    // return new SequentialCommandGroup(
    //   new ParallelCommandGroup(
    //     this.run(() -> setSpeed(speed.get())),
    //     this.run(() -> setRetained(true))
    //   ).withTimeout(1.0),
    //   new ParallelCommandGroup(
    //     () -> {setSpeed(speed.get());
    //     setRetained(false);})
    //   ).withTimeout(1.0)
    // );
  }

  public void setRetained(boolean retained) {
    SmartDashboard.putNumber("Servo Retainer", retained ? 1 : 0);
    finger.set(retained ? ShootakeConstants.kRetainedValue : ShootakeConstants.kNotRetainedValue);
  }

  public boolean hasNote() {
    return !hasNoteSensor.get();
  }

  public Command idleCommand() {
    return this.run(
        () -> {
          setSpeed(0.0);
          setRetained(true);
        });
  }

  public Command retainCommand() {
    return this.run(
            () -> {
              setRetained(true);
            })
        .withTimeout(.25);
  }

  public Command stopCommand() {
    return this.runOnce(() -> this.setSpeed(0));
  }

  public Command intakeCommand() {
    return this.run(
        () -> {
          setRetained(false);
          setSpeed(kIntakeSpeed);
        });
  }

  public Command slowIntakeCommand() {
    return this.run(
        () -> {
          setRetained(false);
          setSpeed(ShootakeConstants.kSlowIntakeSpeed);
        });
  }

  public Command outakeCommand() {
    return this.run(
        () -> {
          setRetained(false);
          setSpeed(ShootakeConstants.kOutakeSpeed);
        });
  }

  public Command ampCommand() {
    return this.run(
        () -> {
          setRetained(false);
          topRoller.set(-.1);
          bottomRoller.set(ShootakeConstants.kAmpSpeed);
        });
  }

  public Command speakerShootCommand() {
    return new SequentialCommandGroup(
        this.run(
                () -> {
                  setRetained(true);
                  setSpeed(-1.0);
                })
            .until(() -> shootDebounce.calculate(getAverageSpeed() > kShootakeFreeSpeed)),
        this.run(
                () -> {
                  setRetained(false);
                  setSpeed(-1.0);
                })
            .until(() -> !hasNote()),
        this.run(
                () -> {
                  setRetained(false);
                  setSpeed(-1.0);
                })
            .withTimeout(.5));
  }
  public Command variableShoot(Supplier<Double> speed) {
    return new SequentialCommandGroup(
        this.run(
                () -> {
                  setRetained(true);
                  setSpeed(-1.0);
                })
            .until(() -> shootDebounce.calculate(getAverageSpeed() > speed.get())),
        this.run(
                () -> {
                  setRetained(false);
                  setSpeed(-1.0);
                })
            .until(() -> !hasNote()),
        this.run(
                () -> {
                  setRetained(false);
                  setSpeed(-1.0);
                })
            .withTimeout(.5));
  }
  /**
   * @deprecated Use {@code speakerShootCommand()} instead.
   */
  public Command shootCommand(BooleanSupplier releaseOverride) {
    return new SequentialCommandGroup(
        new FunctionalCommand(
            () -> {},
            () -> {
              setRetained(true);
              setSpeed(-1.0);
            },
            (Boolean b) -> {},
            () -> getAverageSpeed() > kShootakeFreeSpeed || releaseOverride.getAsBoolean(),
            this),
        new FunctionalCommand(
            () -> {},
            () -> {
              setRetained(false);
              setSpeed(-1.0);
            },
            (Boolean b) -> {},
            () -> getAverageSpeed() < kShootakeLoadSpeedThreshold,
            this));
  }
}
