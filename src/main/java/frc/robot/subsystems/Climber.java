package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import monologue.Logged;

public class Climber extends SubsystemBase implements Logged {
  CANSparkMax leftClimber =
      new CANSparkMax(ClimberConstants.kLeftClimberCANId, MotorType.kBrushless);
  CANSparkMax rightClimber =
      new CANSparkMax(ClimberConstants.kRightClimberCANId, MotorType.kBrushless);

  public Climber() {
    leftClimber.setSmartCurrentLimit(30);
    rightClimber.setSmartCurrentLimit(30);
  }

  public void setSpeedOverride(double speed) {
    leftClimber.set(speed);
    rightClimber.set(-speed);
  }

  public void setSpeed(double speed) {
    if ((leftClimber.getEncoder().getPosition() < ClimberConstants.climbMax && speed > 0)
        || speed < 0) {
      leftClimber.set(speed);
    } else leftClimber.set(0);
    if ((rightClimber.getEncoder().getPosition() * -1 < ClimberConstants.climbMax && speed > 0)
        || speed < 0) {
      rightClimber.set(-speed);
    } else rightClimber.set(0);
    this.log("Climb Speed", speed);
  }

  public Command zero() {
    return this.runOnce(
        () -> {
          rightClimber.getEncoder().setPosition(0);
          leftClimber.getEncoder().setPosition(0);
        });
  }

  public void periodic() {
    this.log("Right Climb Encoder", rightClimber.getEncoder().getPosition() * -1);
    this.log("Left Climb Encoder", leftClimber.getEncoder().getPosition());
  }
}
