package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class Arm extends SubsystemBase implements Logged {
  private CANSparkMax armMotor = new CANSparkMax(ArmConstants.kArmCANId, MotorType.kBrushless);
  private ArmFeedforward armFeedforward =
      new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV); // ks, kg, kv
  private PIDController armPID =
      new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
  private double angleGoal = Math.PI;

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

  public Command holdAngle(double angle) {
    return this.run(
        () ->
            setVoltage(
                armFeedforward.calculate(angle, 0.0) + armPID.calculate(getAngle() - angle)));
  }

  public Command intakePosition() {
    return this.run(
        () -> {
          setVoltage(getAngle() < Math.PI / 2 ? 2.5 : 0.0);
        });
  }

  public Command shootPosition() {
    return this.run(
        () -> {
          setVoltage(getAngle() > Math.PI / 2 ? -2.5 : 0.0);
        });
  }

  public void periodic() {}
}
