package frc.robot.subsystems;

import static frc.robot.Constants.ShootakeConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shootake extends SubsystemBase {
  CANSparkMax topRoller = new CANSparkMax(kTopCANID, MotorType.kBrushless);
  CANSparkMax bottomRoller = new CANSparkMax(kBottomCANID, MotorType.kBrushless);
  Servo finger = new Servo(kFingerPort);

  public Shootake() {
    topRoller.setSmartCurrentLimit(23);
    bottomRoller.setSmartCurrentLimit(23);
  }

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
    return new RunCommand(
        () -> {
          setSpeed(0.0);
          setRetained(true);
        },
        this);
  }

  public Command intakeCommand() {
    return new RunCommand(
        () -> {
          setRetained(false);
          setSpeed(kIntakeSpeed);
        },
        this);
  }

  public Command shootCommand() {
    return new SequentialCommandGroup(
        new FunctionalCommand(
            () -> {},
            () -> {
              setRetained(true);
              setSpeed(-1.0);
            },
            (Boolean b) -> {},
            () -> getAverageSpeed() > kShootakeFreeSpeed,
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

  public void periodic() {
    SmartDashboard.putNumber("Shootake speeds", getAverageSpeed());
  }
}
