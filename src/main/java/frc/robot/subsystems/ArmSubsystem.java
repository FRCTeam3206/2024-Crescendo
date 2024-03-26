package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
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
import frc.robot.Robot;
import monologue.Annotations.Log;
import monologue.LogLevel;
import monologue.Logged;

public class ArmSubsystem extends SubsystemBase implements Logged {

  // Real Arm
  private final CANSparkMax motor;
  private final SparkAbsoluteEncoder encoder;

  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  private final TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              ArmConstants.kMaxVelocity, ArmConstants.kMaxAcceleration));
  private final ArmFeedforward feedforward =
      new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kG, ArmConstants.kA);

  @Log(key = "Feedforward")
  double ff = 0.0;

  private final PIDController feedback = new PIDController(ArmConstants.kP, 0, ArmConstants.kD);

  @Log(key = "Feedback")
  double fb = 0.0;

  // Simulation
  private final PWMSparkMax motorSim;
  private final DutyCycleEncoder dcEncoder;
  private final DutyCycleEncoderSim dcEncoderSim;

  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          ArmConstants.kArmReduction,
          ArmConstants.kArmMOI,
          ArmConstants.kArmLength,
          ArmConstants.kMinAngleRads,
          ArmConstants.kMaxAngleRads,
          true,
          ArmConstants.kMinAngleRads);

  @Log
  private final Mechanism2d mech2d =
      new Mechanism2d(3 * ArmConstants.kArmRealLength, 3 * ArmConstants.kArmRealLength);

  private final MechanismRoot2d mechArmPivot =
      mech2d.getRoot("Pivot", 1.5 * ArmConstants.kArmRealLength, ArmConstants.kArmPivotHeight);

  @SuppressWarnings("unused")
  private final MechanismLigament2d mechArmTower =
      mechArmPivot.append(
          new MechanismLigament2d(
              "Tower", 1.5 * ArmConstants.kArmPivotHeight, -90, 12, new Color8Bit(Color.kBlue)));

  private final MechanismLigament2d mechArm =
      mechArmPivot.append(
          new MechanismLigament2d(
              "Arm",
              ArmConstants.kArmRealLength,
              Units.radiansToDegrees(armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  public ArmSubsystem() {
    if (Robot.isReal()) {
      motor = new CANSparkMax(ArmConstants.kArmCANId, MotorType.kBrushless);
      motor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);
      motor.setIdleMode(IdleMode.kBrake);
      motor.setInverted(false);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

      encoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
      encoder.setZeroOffset(ArmConstants.kArmZeroRads);
      encoder.setPositionConversionFactor(ArmConstants.kPositionConversionFactor);
      // encoder.setVelocityConversionFactor(ArmConstants.kVelocityConversionFactor);
      encoder.setAverageDepth(ArmConstants.kEncoderAveragingDepth);

      // not used for real robot
      motorSim = null;
      dcEncoder = null;
      dcEncoderSim = null;
    } else {
      motorSim = new PWMSparkMax(5);
      dcEncoder = new DutyCycleEncoder(5);
      dcEncoderSim = new DutyCycleEncoderSim(dcEncoder);

      // seed the encoder to have the correct sim arm starting position
      armSim.update(0.020);
      dcEncoderSim.setAbsolutePosition(armSim.getAngleRads());

      // not used for simulation
      motor = null;
      encoder = null;
    }

    feedback.enableContinuousInput(0, 2 * Math.PI);

    this.goal = new TrapezoidProfile.State(getAngle(), 0);
    this.setpoint = new TrapezoidProfile.State(this.goal.position, 0);
  }

  @Override
  public void periodic() {
    super.periodic();

    // the mechanism should track the real or simulated arm position
    mechArm.setAngle(Units.radiansToDegrees(getAngle()));

    // log items that can't be annotated
    this.log("Setpoint Position", setpoint.position);
    this.log("Setpoint Velocity", setpoint.velocity);
    this.log("Goal", this.goal.position);
    this.log(
        "Voltage",
        ((Robot.isReal()) ? motor.get() : motorSim.get()) * RobotController.getBatteryVoltage());
    this.log("Current", (Robot.isReal()) ? motor.getOutputCurrent() : armSim.getCurrentDrawAmps());
  }

  @Override
  public void simulationPeriodic() {
    super.periodic();
    armSim.setInput(motorSim.get() * RobotController.getBatteryVoltage());
    armSim.update(0.020);
    dcEncoderSim.setAbsolutePosition(armSim.getAngleRads());

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
  }

  /**
   * Returns true when the arm is at its current goal and not moving. Tolerances for position and
   * velocity are set in ArmConnstants.
   *
   * @return at goal and not moving
   */
  @Log(level = LogLevel.OVERRIDE_FILE_ONLY) // may want to use on driver dashboard
  public boolean atGoal() {
    return MathUtil.isNear(
            this.goal.position, getAngle(), ArmConstants.kAtAngleTolerance, 0, 2 * Math.PI);
        // && MathUtil.isNear(0, getVelocity(), ArmConstants.kAtVelocityTolerance);
  }

  @Log(key = "Angle")
  public double getAngle() {
    double angle = (Robot.isReal()) ? encoder.getPosition() : dcEncoder.getAbsolutePosition();
    if (angle > ArmConstants.kMaxAngleRads) {
      angle -= 2 * Math.PI;
    }
    return angle;
  }

  @Log(key = "Velocity")
  public double getVelocity() {
    double velocity = (Robot.isReal()) ? encoder.getVelocity() : armSim.getVelocityRadPerSec();
    return velocity;
  }

  private void setVoltage(double voltage) {
    this.log("Requested Voltage", voltage);
    if (Robot.isReal()) {
      motor.setVoltage(voltage);
    } else {
      motorSim.setVoltage(voltage);
    }
  }

  public void moveToGoal(double goal) {
    this.goal =
        new TrapezoidProfile.State(goal, 0); // goal is the desired endpoint with zero velocity
    this.setpoint = profile.calculate(0.020, this.setpoint, this.goal);
    ff = feedforward.calculate(setpoint.position, setpoint.velocity);
    fb = feedback.calculate(getAngle(), setpoint.position);

    setVoltage(fb + ff);
  }

  public void reset() {
    setpoint = new TrapezoidProfile.State(getAngle(), 0); // getVelocity());
    feedback.reset();
  }

  public void stop() {
    setVoltage(0);
  }

  /**
   * Creates a command that will move the arm from its current location to the desired goal and hold
   * it there. This command doesn't exit.
   *
   * @param goal the target angle (radians)
   * @return Command that moves the arm and holds it at goal
   */
  public Command moveToGoalCommand(double goal) {
    return runOnce(this::reset).andThen(run(() -> moveToGoal(goal)));
  }

  /**
   * Creates a command that will move the arm from its current location to the desired goal. This
   * command ends when the arm reaches the goal.
   *
   * @param goal the target angle (radians)
   * @return Command that moves the arm and ends when it reaches the goal
   */
  public Command moveToGoalAndStopCommand(double goal) {
    return moveToGoalCommand(goal).until(this::atGoal);
  }

  /**************************
   * Game-specific commands *
   **************************/

  public Command intakePosition() {
    return moveToGoalCommand(ArmConstants.kIntakeAngle);
  }

  public Command shootPosition() {
    return moveToGoalCommand(ArmConstants.kShootAngle);
  }

  public Command ampPosition() {
    return moveToGoalCommand(ArmConstants.kArmAmpAngle);
  }

  public Command subwooferPosition() {
    return moveToGoalCommand(ArmConstants.kSubwooferAngle);
  }

  public Command ampCommandStop() {
    return ampPosition().until(this::atGoal);
  }

  public Command speakerCommandStop() {
    return shootPosition().until(this::atGoal);
  }

  public Command intakeCommandStop() {
    return intakePosition().until(this::atGoal);
  }
}
