package frc.robot.subsystems;

import static frc.robot.Constants.ShootakeConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShootakeConstants;
import java.util.function.BooleanSupplier;
import monologue.Annotations.Log;
import monologue.Logged;

public class Shootake extends SubsystemBase implements Logged {
  CANSparkMax topRoller = new CANSparkMax(kTopCANID, MotorType.kBrushless);
  CANSparkMax bottomRoller = new CANSparkMax(kBottomCANID, MotorType.kBrushless);
  Servo finger = new Servo(kFingerPort);

  public Shootake() {
    topRoller.setSmartCurrentLimit(23);
    bottomRoller.setSmartCurrentLimit(23);
  }

  @Log
  private double getAverageSpeed() {
    return -(topRoller.getEncoder().getVelocity() + bottomRoller.getEncoder().getVelocity()) / 2;
  }

  public void setSpeed(double speed) {
    topRoller.set(speed);
    bottomRoller.set(speed);
  }

  public void setRetained(boolean retained) {
    SmartDashboard.putNumber("Servo Retainer", retained ? 1 : 0);
    finger.set(retained ? 1.0 : .6);
  }

  public Command idleCommand() {
    return this.run(
        () -> {
          setSpeed(0.0);
          setRetained(true);
        });
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
  public Command ampCommand(){
    return this.run(
      ()->{
        setRetained(false);
        topRoller.set(0);
        bottomRoller.set(ShootakeConstants.kOutakeSpeed);
      }
    );
  }
  public Command shootCommand(BooleanSupplier releaseOverride) {
    return new SequentialCommandGroup(
        new FunctionalCommand(
            () -> {},
            () -> {
              setRetained(true);
              setSpeed(-1.0);
            },
            (Boolean b) -> {},
            () ->
                (getAverageSpeed() > kShootakeFreeSpeed || releaseOverride.getAsBoolean())
                    || (getAverageSpeed() > kShootakeFreeSpeed && releaseOverride.getAsBoolean()),
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

  public void periodic() {}
}
