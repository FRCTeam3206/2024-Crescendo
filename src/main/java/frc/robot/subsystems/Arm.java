package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPostition;
import monologue.Annotations.Log;
import monologue.Logged;

public class Arm extends SubsystemBase implements Logged {
  private CANSparkMax armMotor = new CANSparkMax(ArmConstants.kArmCANId, MotorType.kBrushless);
  private ArmFeedforward armFeedforward =
      new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV); // ks, kg, kv
  private PIDController armPID =
      new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
  private double angleGoal = Math.PI;
  private ArmPostition position = ArmPostition.SHOOT;

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
    return (rawAngle - ArmConstants.kArmZeroOffset) * (2 * Math.PI) + (Math.PI / 2);
    // return 2 * Math.PI * Math.abs(armMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition() -
    // 0.25);
  }

  public boolean atSpeakerAngle() {
    return MathUtil.applyDeadband(
            getAngle() - ArmConstants.kShootAngle, ArmConstants.kAtAngleTolerance)
        == 0.0;
  }

  public boolean atAmpAngle() {
    return MathUtil.applyDeadband(
            getAngle() - ArmConstants.kArmAmpAngle, ArmConstants.kAtAngleTolerance)
        == 0.0;
  }



  public boolean atIntakeAngle() {
    return MathUtil.applyDeadband(
            getAngle() - ArmConstants.kShootAngle, ArmConstants.kAtAngleTolerance)
        == 0.0;
  }

  public Command intakePosition() {
    return this.run(
        () -> {
          double moveVoltage =
              getAngle() < Math.PI / 2 + ArmConstants.kArmZeroThreshold ? 2.5 : 0.0;
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
              getAngle() > Math.PI / 2 - ArmConstants.kArmZeroThreshold ? -2.5 : 0.0;
          setVoltage(appliedVoltage);
        });
  }

  public Command ampPosition() {
    return this.run(
        () -> {
          armPID.setSetpoint(ArmConstants.kArmAmpAngle);
          double pid = armPID.calculate(getAngle());
          double ff = Math.cos(getAngle()) * ArmConstants.kG;
          double voltage = pid + ff;
          voltage = MathUtil.clamp(voltage, -2.5, 2.5);
          setVoltage(voltage);
        });
  }
  public Command subwooferPosition() {
    return this.run(
        () -> {
          armPID.setSetpoint(1.380);//TODO Constant
          double pid = armPID.calculate(getAngle());
          double ff = Math.cos(getAngle()) * ArmConstants.kG;
          double voltage = pid + ff;
          voltage = MathUtil.clamp(voltage, -2.5, 2.5);
          setVoltage(voltage);
        });
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
