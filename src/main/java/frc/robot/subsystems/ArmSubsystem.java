package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Robot;
import monologue.Annotations.Log;
import monologue.Logged;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmSubsystem extends SubsystemBase implements Logged {

  private CANSparkMax m_motor = new CANSparkMax(ArmConstants.kArmCANId, MotorType.kBrushless);
  private final SparkAbsoluteEncoder m_armEncoder;
  private final SparkPIDController m_armPIDController;

  private final DutyCycleEncoder m_armDCEncoder = new DutyCycleEncoder(5);
  private final DutyCycleEncoderSim m_simArmEncoder = new DutyCycleEncoderSim(m_armDCEncoder);

  @Log private double m_armSetpointRads = 4 * Math.PI / 4;
  @Log private double m_armKp = ArmConstants.kPSpark;
  @Log private double armAngle;

  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts, ArmConstants.kGVolts,
          ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  private final PIDController m_simplePID = new PIDController(m_armKp, 0, 0);

  private final ProfiledPIDController m_armProfiledPID =
      new ProfiledPIDController(
          m_armKp,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ArmConstants.kMaxVelocityRadPerSecond,
              ArmConstants.kMaxAccelerationRadPerSecSquared));

  private final PWMSparkMax m_simMotor = new PWMSparkMax(5);
  private final DCMotor m_simGearBox = DCMotor.getNEO(1);

  // private final EncoderSim m_simEncoder;
  private final SingleJointedArmSim m_simArm =
      new SingleJointedArmSim(
          m_simGearBox,
          ArmConstants.kArmReduction,
          SingleJointedArmSim.estimateMOI(ArmConstants.kArmLength, ArmConstants.kArmMass),
          ArmConstants.kArmLength,
          ArmConstants.kMinAngleRads,
          ArmConstants.kMaxAngleRads,
          true,
          0);

  @Log
  private final Mechanism2d m_mech2d =
      new Mechanism2d(3 * ArmConstants.kArmLength, 3 * ArmConstants.kArmLength);

  private final MechanismRoot2d m_armPivot =
      m_mech2d.getRoot("Pivot", 1.5 * ArmConstants.kArmLength, 1.5 * ArmConstants.kArmLength);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("Tower", 1.5 * ArmConstants.kArmLength, -90));
  private final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              ArmConstants.kArmLength,
              Units.radiansToDegrees(m_simArm.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  /** Create a new ArmSubsystem. */
  public ArmSubsystem() {

    m_motor.setSmartCurrentLimit(45);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(true);

    m_armEncoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);
    m_armEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_armEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
    m_armEncoder.setZeroOffset(ArmConstants.kArmZeroRads);
    // m_simEncoder = new EncoderSim(m_armEncoder);

    m_armPIDController = m_motor.getPIDController();
    m_armPIDController.setFeedbackDevice(m_armEncoder);
    m_armPIDController.setP(ArmConstants.kPSpark);
    m_armPIDController.setI(0);
    m_armPIDController.setD(0);
    m_armPIDController.setPositionPIDWrappingEnabled(true);
    // m_armPIDController.setPositionPIDWrappingMaxInput();

    m_armTower.setColor(new Color8Bit(Color.kBlue));

    // Preferences.initDouble(ArmConstants.kArmPositionKey, m_armSetpointRads);
    // Preferences.initDouble(ArmConstants.kArmPKey, m_armKp);
  }

  public void simulationPeriodic() {
    m_simArm.setInput(m_simMotor.get() * RobotController.getBatteryVoltage());
    m_simArm.update(0.020);
    m_simArmEncoder.setAbsolutePosition(m_simArm.getAngleRads());

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_simArm.getCurrentDrawAmps()));
    m_arm.setAngle(Units.radiansToDegrees(m_simArm.getAngleRads()));
    this.log("Simulated Voltage", m_simMotor.get() * RobotController.getBatteryVoltage());
    this.log("Simulated Current", m_simArm.getCurrentDrawAmps());
  }

  @Log
  public double getAngleRads() {
    if (Robot.isReal()) {
      return m_armEncoder.getPosition();
    } else {
      return m_armDCEncoder.getAbsolutePosition();
    }
  }

  public void moveToGoal(double goal) {
    var ff = m_feedforward.calculate(getAngleRads(), 0);
    this.log("FeedForward", ff);
    var pidOutput = ff + m_armProfiledPID.calculate(getAngleRads(), goal);
    this.log("Setpoint Position", m_armProfiledPID.getSetpoint().position);
    pidOutput = MathUtil.clamp(pidOutput, -6, 6);
    this.log("PID Output Voltage", pidOutput);
    m_motor.setVoltage(pidOutput);
    m_simMotor.setVoltage(pidOutput);
  }

  public Command moveToGoalCommand(double goal) {
    return this.run(() -> moveToGoal(goal));
  }

  public void reset() {
    if (Robot.isSimulation()) {
      m_armProfiledPID.reset(m_simArm.getAngleRads());
    }
  }

  public void stop() {
    m_motor.setVoltage(0);
    m_simMotor.setVoltage(0);
  }

  // @Override
  // public void useState(TrapezoidProfile.State setpoint) {
  //   // Calculate the feedforward from the sepoint
  //   this.log("setpoint.position", setpoint.position);
  //   this.log("setpoint.velocity", setpoint.velocity);

  //   double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
  //   // Add the feedforward to the PID output to get the motor output
  //   this.log("FeedForward", feedforward);

  //   m_armPIDController.setReference(
  //       setpoint.position, CANSparkMax.ControlType.kPosition, 0, feedforward);
  // }

  // public Command setArmGoalCommand(double kArmOffsetRads) {
  //   return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
  // }

  /** Load setpoint and kP from preferences. */
  // public void loadPreferences() {
  //   // Read Preferences for Arm setpoint and kP on entering Teleop
  //   m_armSetpointRads = Preferences.getDouble(ArmConstants.kArmPositionKey, m_armSetpointRads);
  //   if (m_armKp != Preferences.getDouble(ArmConstants.kArmPKey, m_armKp)) {
  //     m_armKp = Preferences.getDouble(ArmConstants.kArmPKey, m_armKp);
  //     m_simplePID.setP(m_armKp);
  //   }
  // }

  // public Command loadPreferencesCommand() {
  //   return runOnce(this::loadPreferences);
  // }

  @Override
  public void periodic() {
    super.periodic();
    armAngle = m_armEncoder.getPosition();
  }
}
