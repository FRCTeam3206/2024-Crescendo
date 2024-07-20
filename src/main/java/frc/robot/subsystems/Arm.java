package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
    if (rawAngle > 0.75){
      rawAngle = rawAngle -1.0;
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

  public Command higherPosition(Command finalAngleCommand) {
    return new ConditionalCommand(finalAngleCommand, new ConditionalCommand(this.run(() -> setVoltage(2.5)), this.run(() -> setVoltage(-2.5)), () -> getAngle() < Math.PI / 2), () -> getAngle() > Math.PI / 4 && getAngle() < 3 * Math.PI / 4);
  }

  public Command ampPositionFinish() {
    return this.run(
        () -> {
          armPID.setSetpoint(ArmConstants.kArmAmpAngle);
          double pid = armPID.calculate(getAngle());
          double ff = Math.cos(getAngle()) * ArmConstants.kG;
          double voltage = pid + ff;
          voltage = MathUtil.clamp(voltage, -3.0, 3.0);
          setVoltage(voltage);
        });
  }

  public Command ampPosition() {
    return higherPosition(ampPositionFinish());
    //return new ConditionalCommand(ampPositionFinish(), new ConditionalCommand(this.run(() -> , () -> , () -> getAngle() > ArmConstants.kArmAmpAngle)), () -> MathUtil.isNear(ArmConstants.kArmAmpAngle, getAngle(), 0.4));
  }

  public Command subwooferPositionFinish() {
    return this.run(
        () -> {
          armPID.setSetpoint(ArmConstants.kSubwooferAngle);
          double pid = armPID.calculate(getAngle());
          double ff = Math.cos(getAngle()) * ArmConstants.kG;
          double voltage = pid + ff;
          voltage = MathUtil.clamp(voltage, -3.0, 3.0);
          setVoltage(voltage);
        });
  }

  public Command subwooferPosition() {
    return higherPosition(subwooferPositionFinish());
  }

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
