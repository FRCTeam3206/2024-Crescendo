package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class Arm extends SubsystemBase implements Logged {
  private CANSparkMax armMotor = new CANSparkMax(ArmConstants.kArmCANId, MotorType.kBrushless);

  public Arm() {
    armMotor.setSmartCurrentLimit(45);
  }

  public void setVoltage(double voltage) {
    SmartDashboard.putNumber("Arm Motor Voltage", voltage);
    armMotor.setVoltage(voltage);
  }

  @Log.NT
  public double getAngle() {
    double rawAngle = armMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    this.log("Raw Angle", rawAngle);
    return (rawAngle - ArmConstants.kArmZeroOffset) * (2 * Math.PI) + (Math.PI / 2);
    // return 2 * Math.PI * Math.abs(armMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition() -
    // 0.25);
  }

  public void periodic() {}
}
