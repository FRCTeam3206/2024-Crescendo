// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.sensors.AprilTagVision;
import frc.utils.SwerveUtils;
import java.io.IOException;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;

public class DriveSubsystem extends SubsystemBase implements Logged {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft =
      new MAXSwerveModule(
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight =
      new MAXSwerveModule(
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft =
      new MAXSwerveModule(
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight =
      new MAXSwerveModule(
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition(),
          },
          new Pose2d());

  private final AprilTagVision poseCamera1 =
      new AprilTagVision("Camera1", VisionConstants.kDistToCamera1, m_poseEstimator);

  private Pose2d simOdometryPose = m_poseEstimator.getEstimatedPosition();

  SwerveModuleState[] m_desiredStates =
      new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
      };

  private Field2d m_field = new Field2d();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SmartDashboard.putData("Field", m_field);

    AutoBuilder.configureHolonomic(
        this::getPose,
        (Pose2d pose) -> {
          resetOdometry(pose);
        },
        this::getRobotRelativeChassisSpeeds,
        (ChassisSpeeds speeds) -> {
          drive(
              speeds.vxMetersPerSecond,
              speeds.vyMetersPerSecond,
              speeds.omegaRadiansPerSecond,
              false,
              true);
        },
        new HolonomicPathFollowerConfig(
            PathPlannerConstants.translationPID,
            PathPlannerConstants.rotationPID,
            AutoConstants.kMaxModuleSpeedMetersPerSecond,
            Math.sqrt(
                    Math.pow(DriveConstants.kTrackWidth, 2)
                        + Math.pow(DriveConstants.kWheelBase, 2))
                / 2,
            new ReplanningConfig()),
        () -> {
          return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        },
        this);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    updateOdometry();
    m_field.setRobotPose(getPose());
    Pose2d redGoal = new Pose2d(13.349, 5.326, new Rotation2d());
    Pose2d blueGoal = new Pose2d(3.110, 5.326, new Rotation2d());
    double redDist = getPose().getTranslation().getDistance(redGoal.getTranslation());
    double blueDist = getPose().getTranslation().getDistance(blueGoal.getTranslation());
    double dist = Math.min(redDist, blueDist);
    SmartDashboard.putBoolean("Can Shoot", dist < .15);
    SmartDashboard.putNumber("Blue Dist", blueDist);
    SmartDashboard.putNumber("Red Dist", redDist);

    try {
      double driverDirection = 0;
      double angleTowardGoal;
      double xError;
      double yError;
      // double driverRelativeOffset;
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        xError = blueGoal.getX() - getPose().getX();
        yError = getPose().getY() - blueGoal.getY();
        // Math.atan((blueGoal.getX()) / 1.0)
        // driverRelativeOffset
      } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        xError = getPose().getX() - redGoal.getX();
        yError = redGoal.getY() - getPose().getY();
      } else {
        throw new IOException("Alliance is neither red nor blue.");
      }

      double rawArcTanValue = Math.atan(xError / yError);
      angleTowardGoal =
          rawArcTanValue > 0
              ? yError > 0 ? rawArcTanValue : rawArcTanValue + Math.PI
              : yError > 0 ? rawArcTanValue + (2 * Math.PI) : rawArcTanValue + Math.PI;
      // Make zero up instead of right (as gyro shown on Shuffleboard instead of as it would look on
      // the unit circle)
      angleTowardGoal = (angleTowardGoal - (Math.PI / 2)) % (2 * Math.PI);
      // In degrees for displaying as gyro on Shuffleboard.
      angleTowardGoal *= (180 / Math.PI);
      driverDirection = angleTowardGoal;
      SmartDashboard.putNumber("Direction To Shoot Pose", driverDirection);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  @Log
  public Pose2d getPose() {
    return (Robot.isReal()) ? m_poseEstimator.getEstimatedPosition() : simOdometryPose;
  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        new SwerveModuleState[] {
          m_frontLeft.getState(),
          m_frontRight.getState(),
          m_rearLeft.getState(),
          m_rearRight.getState()
        });
  }

  /**
   * Updates the odometry (including for sim). Should be called from periodic(). This code comes
   * from Team 2713:
   * https://github.com/FRC2713/Robot2022-v2/blob/main/src/main/java/frc/robot/subsystems/SwerveIO/BabySwerver.java#L126-151
   */
  private void updateOdometry() {
    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

    poseCamera1.addVisionMeasurementToEstimator();

    SwerveModuleState[] measuredStates =
        new SwerveModuleState[] {
          m_frontLeft.getState(),
          m_frontRight.getState(),
          m_rearLeft.getState(),
          m_rearRight.getState()
        };

    this.log("Desired States", m_desiredStates);
    this.log("Current States", measuredStates);

    // if (Robot.isSimulation()) {
    //   double timeDelta = 0.020; // standard loop time is 20 ms
    //   ChassisSpeeds speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(measuredStates);
    //   simOdometryPose =
    //       simOdometryPose.exp(
    //           new Twist2d(
    //               speeds.vxMetersPerSecond * timeDelta,
    //               speeds.vyMetersPerSecond * timeDelta,
    //               speeds.omegaRadiansPerSecond * timeDelta));
    // }
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
    simOdometryPose = pose;
  }

  public void resetDirection() {
    resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d(0.0)));
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit Whether to enable rate limiting for smoother control.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate =
            500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir =
            SwerveUtils.StepTowardsCircular(
                m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag
            > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir =
            SwerveUtils.StepTowardsCircular(
                m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var desiredStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered, ySpeedDelivered, rotDelivered, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    // m_desiredStates = swerveModuleStates;
    setModuleStates(desiredStates);
    //             SwerveDriveKinematics.desaturateWheelSpeeds(
    //     swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    // m_frontLeft.setDesiredState(swerveModuleStates[0]);
    // m_frontRight.setDesiredState(swerveModuleStates[1]);
    // m_rearLeft.setDesiredState(swerveModuleStates[2]);
    // m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Creates a command that will drive the robot with dynamic inputs for x, y, and angular speeds.
   *
   * <p>Speed suppliers should be specified as lambdas or method references. If joystick inputs need
   * to have a deadband or input squaring function applied, do so within the lambda function.
   *
   * @param xSpeed provides speeds in the X direction (forward).
   * @param ySpeed provides speeds in the Y direction (sideways).
   * @param rot provides angular speeds.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit Whether to enable rate limiting for smoother control.
   * @return the driving command
   * @see <a href=
   *     "https://docs.wpilib.org/en/stable/docs/software/basic-programming/functions-as-data.html">
   *     https://docs.wpilib.org/en/stable/docs/software/basic-programming/functions-as-data.html
   *     </a>
   */
  public Command driveCommand(
      DoubleSupplier xSpeed,
      DoubleSupplier ySpeed,
      DoubleSupplier rot,
      BooleanSupplier fieldRelative,
      boolean rateLimit) {
    return this.run(
        () -> {
          drive(
              xSpeed.getAsDouble(),
              ySpeed.getAsDouble(),
              rot.getAsDouble(),
              fieldRelative.getAsBoolean(),
              rateLimit);
        });
  }

  public Command stopCommand() {
    return this.runOnce(() -> drive(0, 0, 0, true, true));
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    var desiredStates =
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        };
    setModuleStates(desiredStates);
    // m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    // m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    // m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    // m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /** Creates a command that continually sets the wheels into an X formation to prevent movement. */
  public Command setXCommand() {
    return this.run(this::setX);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
    m_desiredStates = desiredStates;
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  public Command zeroHeadingCommand() {
    return this.runOnce(this::zeroHeading).ignoringDisable(true).withName("Reset Gyro");
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  @Log
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /** Update the gyro when simulating the robot. */
  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble gyroSimAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    gyroSimAngle.set(-getPose().getRotation().getDegrees());

    double timeDelta = 0.020; // standard loop time is 20 ms
    ChassisSpeeds speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(m_desiredStates);
    simOdometryPose =
        simOdometryPose.exp(
            new Twist2d(
                speeds.vxMetersPerSecond * timeDelta,
                speeds.vyMetersPerSecond * timeDelta,
                speeds.omegaRadiansPerSecond * timeDelta));
  }

  public Command pathCommandToPose(Pose2d goalPose) {
    return AutoBuilder.pathfindToPose(
        goalPose,
        new PathConstraints(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
            AutoConstants.kMaxAngularSpeedRadiansPerSecond,
            AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));
  }

  public static final class PathPlannerConstants {
    public static final PIDConstants translationPID =
        new PIDConstants(1.0, 0.0, 0.0); // TODO Find value
    public static final PIDConstants rotationPID =
        new PIDConstants(1.0, 0.0, 0.0); // TODO Find value
  }
}
