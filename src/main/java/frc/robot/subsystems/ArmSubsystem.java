package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ModuleConstants;
import monologue.Logged;
import monologue.Annotations.Log;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmSubsystem extends TrapezoidProfileSubsystem implements Logged{

  private CANSparkMax m_motor = new CANSparkMax(ArmConstants.kArmCANId, MotorType.kBrushless);
  private final SparkAbsoluteEncoder m_armEncoder;
  private final SparkPIDController m_armPIDController;

  @Log private double armAngle;

  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts, ArmConstants.kGVolts,
          ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  /** Create a new ArmSubsystem. */
  public ArmSubsystem() {
    super(
        new TrapezoidProfile.Constraints(
            ArmConstants.kMaxVelocityRadPerSecond, ArmConstants.kMaxAccelerationRadPerSecSquared),
        ArmConstants.kArmOffsetRads);
    
    m_motor.setSmartCurrentLimit(45);
    m_motor.setIdleMode(IdleMode.kBrake);

    m_armEncoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);
    m_armEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_armEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
    m_armEncoder.setZeroOffset(ArmConstants.kArmOffsetRads);

    m_armPIDController = m_motor.getPIDController();
    m_armPIDController.setFeedbackDevice(m_armEncoder);
    m_armPIDController.setP(ArmConstants.kPSpark);
    m_armPIDController.setI(0);
    m_armPIDController.setD(0);
  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    this.log("setpoint.position", setpoint.position);
    this.log("setpoint.velocity", setpoint.velocity);

    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output

    m_armPIDController.setReference(setpoint.position, CANSparkMax.ControlType.kPosition, 0, feedforward);
  }

  public Command setArmGoalCommand(double kArmOffsetRads) {
    return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
  }

  @Override
  public void periodic() {
    armAngle = m_armEncoder.getPosition();
  }
}
