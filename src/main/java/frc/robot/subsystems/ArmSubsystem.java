package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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

  @Log private double m_armSetpointRads = 4 * Math.PI / 4;
  @Log private double m_armKp = ArmConstants.kPSpark;
  @Log private double armAngle;

  @Log private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  @Log private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();

  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts, ArmConstants.kGVolts,
          ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  private final PIDController m_pid = new PIDController(m_armKp, 0, 0);

  private final TrapezoidProfile m_profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              ArmConstants.kMaxVelocityRadPerSecond,
              ArmConstants.kMaxAccelerationRadPerSecSquared));

  // These are used for the arm simulator
  private final DutyCycleEncoder m_armDCEncoder = new DutyCycleEncoder(5);
  private final DutyCycleEncoderSim m_simArmEncoder = new DutyCycleEncoderSim(m_armDCEncoder);

  private final PWMSparkMax m_simMotor = new PWMSparkMax(5);
  private final DCMotor m_simGearBox = DCMotor.getNEO(1);

  private final SingleJointedArmSim m_simArm =
      new SingleJointedArmSim(
          m_simGearBox,
          ArmConstants.kArmReduction,
          ArmConstants.kArmMOI,
          ArmConstants.kArmLength,
          ArmConstants.kMinAngleRads,
          ArmConstants.kMaxAngleRads,
          true,
          0,
          VecBuilder.fill(
              Units.rotationsToRadians(1.0 / 10024)) // Add noise with a std-dev of 0.5 degrees
          );

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
    m_armEncoder.setAverageDepth(8);
    m_pid.enableContinuousInput(0, 2 * Math.PI);

    m_armTower.setColor(new Color8Bit(Color.kBlue));
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
  public double getAngle() {
    if (Robot.isReal()) {
      return m_armEncoder.getPosition();
    } else {
      return m_armDCEncoder.getAbsolutePosition();
    }
  }

  @Log
  public double getVelocity() {
    if (Robot.isReal()) {
      return m_armEncoder.getVelocity();
    } else {
      return m_simArm.getVelocityRadPerSec();
    }
  }

  public void moveToGoal(double goal) {
    m_goal = new TrapezoidProfile.State(goal, 0); // target is to be stationary at angle "goal"

    m_setpoint =
        m_profile.calculate(
            0.020, m_setpoint, m_goal); // get updated setpoint from trapezoidal profile

    this.log("Setpoint Position", m_setpoint.position);
    this.log("Setpoint Velocity", m_setpoint.velocity);

    double ff = m_feedforward.calculate(m_setpoint.position, m_setpoint.velocity);
    this.log("FeedForward", ff);

    double pid = m_pid.calculate(getAngle(), m_setpoint.position);
    this.log("PID", pid);

    double output = pid + ff;
    this.log("PID Output Voltage", output);
    m_motor.setVoltage(output);
    m_simMotor.setVoltage(output);
  }

  public Command moveToGoalCommand(double goal) {
    return this.run(() -> moveToGoal(goal));
  }

  public void reset() {
    m_setpoint = new TrapezoidProfile.State(getAngle(), getVelocity());
    m_pid.reset();
  }

  public void stop() {
    m_motor.setVoltage(0);
    m_simMotor.setVoltage(0);
  }

  public Command intakePosition() {
    return this.runOnce(this::reset).andThen(this.run(() -> moveToGoal(ArmConstants.kIntakeAngle)));
  }

  public Command shootPosition() {
    return this.runOnce(this::reset).andThen(this.run(() -> moveToGoal(ArmConstants.kShootAngle)));
  }

  public Command ampPosition() {
    return this.runOnce(this::reset).andThen(this.run(() -> moveToGoal(ArmConstants.kArmAmpAngle)));
  }

  public Command subwooferPosition() {
    return this.runOnce(this::reset).andThen(this.run(() -> moveToGoal(ArmConstants.kSubwooferAngle)));
  }

  public boolean atSpeakerAngle() {
    return Math.abs(getAngle() - ArmConstants.kShootAngle) < ArmConstants.kAtAngleTolerance;
  }

  public boolean atAmpAngle() {
    return Math.abs(getAngle() - ArmConstants.kArmAmpAngle) < ArmConstants.kAtAngleTolerance;
  }

  public boolean atIntakeAngle() {
    return Math.abs(getAngle() - ArmConstants.kIntakeAngle) < ArmConstants.kAtAngleTolerance;
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

  @Override
  public void periodic() {
    super.periodic();
    armAngle = m_armEncoder.getPosition();
  }
}
