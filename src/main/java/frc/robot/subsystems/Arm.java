package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class Arm extends SubsystemBase implements Logged {
  private CANSparkMax armMotor = new CANSparkMax(ArmConstants.kArmCANId, MotorType.kBrushless);
  private PIDController armPID =
      new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);

  public Arm() {
    armMotor.setSmartCurrentLimit(45);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
  }

  public void setVoltage(double voltage) {
    SmartDashboard.putNumber("Arm Motor Voltage", voltage);
    armMotor.setVoltage(voltage);
  }

  @Log
  public double getAngle() {
    double rawAngle = armMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    this.log("Raw Angle", rawAngle);
    rawAngle = rawAngle - ArmConstants.kArmZeroOffset;
    if (rawAngle > 0.75) {
      rawAngle = rawAngle - 1.0;
    }
    return rawAngle * (2 * Math.PI) + (Math.PI / 2);
    // return 2 * Math.PI * Math.abs(armMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition() -
    // 0.25);
  }

  @Log
  public boolean atSpeakerAngle() {
    return Math.abs(getAngle() - ArmConstants.kShootAngle) < ArmConstants.kAtAngleTolerance;
  }

  public boolean atAmpAngle() {
    return Math.abs(getAngle() - ArmConstants.kArmAmpAngle) < ArmConstants.kAtAngleTolerance;
  }

  public boolean atIntakeAngle() {
    return Math.abs(getAngle() - ArmConstants.kIntakeAngle) < ArmConstants.kAtAngleTolerance;
  }

  public Command intakePosition() {
    return this.run(
        () -> {
          double moveVoltage =
              Math.abs(getAngle() - ArmConstants.kIntakeAngle) > ArmConstants.kArmZeroThreshold
                  ? 2.5
                  : 0.0;
          if (Math.abs(getAngle() - 3.5) < .05) {
            setVoltage(0);
          } else {
            setVoltage(moveVoltage);
          }
        });
  }

  public Command shootPosition() {
    return this.run(
        () -> {
          double appliedVoltage =
              Math.abs(getAngle() - ArmConstants.kShootAngle) > ArmConstants.kArmZeroThreshold
                  ? -2.5
                  : 0.0;
          setVoltage(appliedVoltage);
        });
  }

  @Log
  public boolean inIntermediateRange() {
    return getAngle() > Math.PI / 4 && getAngle() < 3 * Math.PI / 4;
  }

  public void goTowardsIntermediate() {
    if (getAngle() < Math.PI / 2) {
      setVoltage(2.5);
    } else {
      setVoltage(-2.5);
    }
  }

  // public Command higherPosition(Command finalAngleCommand) {
  //   return new ConditionalCommand(
  //       finalAngleCommand,
  //       new ConditionalCommand(
  //           this.run(() -> setVoltage(2.5)),
  //           this.run(() -> setVoltage(-2.5)),
  //           () -> getAngle() < Math.PI / 2),
  //       () -> inIntermediateRange());
  // }

  public Command ampPosition() {
    return this.run(
        () -> {
          if (inIntermediateRange()) {
            armPID.setSetpoint(ArmConstants.kArmAmpAngle);
            double pid = armPID.calculate(getAngle());
            double ff = Math.cos(getAngle()) * ArmConstants.kG;
            double voltage = pid + ff;
            voltage = MathUtil.clamp(voltage, -3.0, 3.0);
            setVoltage(voltage);
          } else {
            goTowardsIntermediate();
          }
        });
  }

  // public Command ampPosition() {
  //   return higherPosition(ampPositionFinish());
  // }

  public Command subwooferPosition() {
    return this.run(
        () -> {
          if (inIntermediateRange()) {
            armPID.setSetpoint(ArmConstants.kSubwooferAngle);
            double pid = armPID.calculate(getAngle());
            double ff = Math.cos(getAngle()) * ArmConstants.kG;
            double voltage = pid + ff;
            voltage = MathUtil.clamp(voltage, -3.0, 3.0);
            setVoltage(voltage);
          } else {
            goTowardsIntermediate();
          }
        });
  }

  // public Command subwooferPosition() {
  //   return higherPosition(subwooferPositionFinish());
  // }

  public Command intakeCommandStop() {
    return intakePosition().until(() -> atIntakeAngle());
  }

  public Command ampCommandStop() {
    return ampPosition().until(() -> atAmpAngle());
  }

  public Command speakerCommandStop() {
    return shootPosition().until(() -> atSpeakerAngle());
  }

  public void periodic() {}
}
