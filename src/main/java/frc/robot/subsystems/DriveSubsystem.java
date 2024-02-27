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
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AllianceNoteLocation;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RelativeTo;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer.AllianceColor;
import frc.robot.Robot;
import frc.robot.sensors.AprilTagVision;
import frc.utils.AllianceUtil;
import frc.utils.SwerveUtils;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
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
      new AprilTagVision(
          VisionConstants.kCameraName1, VisionConstants.kDistToCamera1, m_poseEstimator);
  private final AprilTagVision poseCamera2 =
      new AprilTagVision(
          VisionConstants.kCameraName2, VisionConstants.kDistToCamera2, m_poseEstimator);

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

    AllianceUtil.setRobot(this::getPose);

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
              RelativeTo.ROBOT_RELATIVE,
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
    // this.log("Angle from speaker",
    // getAngleFromPointPositive(AllianceUtil.getPoseForAlliance(AutoAlignConstants.kBlueSpeakerPose)));
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
    poseCamera2.addVisionMeasurementToEstimator();

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
      double xSpeed, double ySpeed, double rot, RelativeTo relativeTo, boolean rateLimit) {
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

    SwerveModuleState[] desiredStates;
    switch (relativeTo) {
      case ROBOT_RELATIVE:
        desiredStates =
            DriveConstants.kDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        break;
      case DRIVER_RELATIVE:
        desiredStates =
            DriveConstants.kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered, ySpeedDelivered, rotDelivered, m_gyro.getRotation2d()));
        break;
      case FIELD_RELATIVE:
        desiredStates =
            DriveConstants.kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered, ySpeedDelivered, rotDelivered, getPose().getRotation()));
        break;
        // This should be an impossible case, but it gives an error otherwise.
      default:
        desiredStates =
            DriveConstants.kDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    }
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
      Supplier<RelativeTo> relativeTo,
      boolean rateLimit) {
    return this.run(
        () -> {
          drive(
              xSpeed.getAsDouble(),
              ySpeed.getAsDouble(),
              rot.getAsDouble(),
              relativeTo.get(),
              rateLimit);
        });
  }

  public void stop() {
    drive(0, 0, 0, RelativeTo.FIELD_RELATIVE, true);
  }

  public Command stopCommand() {
    return this.runOnce(() -> stop());
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

  public void resetGryoToVision(){
    if(AllianceUtil.getAlliance()==AllianceColor.BLUE){
      m_gyro.setAngleAdjustment(-getPose().getRotation().getDegrees());
    }else if(AllianceUtil.getAlliance()==AllianceColor.RED){
      m_gyro.setAngleAdjustment(-getPose().getRotation().getDegrees()+180);
    }
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

  // public double distBetweenPoses(Pose2d pose1, Pose2d pose2) {
  //   return Math.sqrt(
  //       Math.pow(pose1.getX() - pose2.getX(), 2) + Math.pow(pose1.getY() - pose2.getY(), 2));
  // }

  public double getAngleToGoal(Rotation2d angleGoal) {
    return angleGoal.minus(getPose().getRotation()).getRadians();
  }

  public double getAngleToGoal(double angleGoal) {
    return getAngleToGoal(Rotation2d.fromRadians(angleGoal));
  }

  public boolean isAtGoal(Pose2d goalPose) {
    return getPose().getTranslation().getDistance(goalPose.getTranslation())
            < AutoAlignConstants.kAtGoalTolerance
        && Math.abs(getAngleToGoal(goalPose.getRotation()))
            < AutoAlignConstants.kAtRotationGoalTolerance;
  }

  public boolean isAtDistFromPoint(Pose2d refPoint, double goalDist) {
    Pose2d currentPose = getPose();
    return Math.abs(currentPose.getTranslation().getDistance(refPoint.getTranslation()) - goalDist)
        < AutoAlignConstants.kAtGoalTolerance;
  }

  /**
   * @param refPose The position being referenced to figure out the angle to.
   * @return The angle from the current position to the reference position (if a circle is drawn to
   *     show this, the current pose would be the center). In the range of -pi to pi.
   */
  public double getAngleToPoint(Pose2d refPose) {
    Pose2d currentPose = getPose();
    return Math.atan2(refPose.getY() - currentPose.getY(), refPose.getX() - currentPose.getX());
  }

  /**
   * @param refPose The position being referenced to figure out the angle to.
   * @return The angle from the reference position to the current position (if a circle is drawn to
   *     show this, the reference pose would be the center). In the rangle of 0 to two pi.
   */
  public double getAngleFromPointPositive(Pose2d refPose) {
    return getAngleToPoint(refPose) + Math.PI;
  }

  public Command driveToGoalCommand(Supplier<Pose2d> goalPose) {
    return this.run(() -> driveToGoal(goalPose.get()))
        .until(
            () -> {
              return Math.abs(goalPose.get().getX() - getPose().getX())
                      < AutoAlignConstants.kAtGoalTolerance
                  && Math.abs(goalPose.get().getY() - getPose().getY())
                      < AutoAlignConstants.kAtGoalTolerance
                  && this.getAngleToGoal(goalPose.get().getRotation())
                      < AutoAlignConstants.kAtRotationGoalTolerance;
            });
  }

  public void driveToGoal(Pose2d goalPose) {
    Pose2d currentPose = getPose();
    double deltaX = goalPose.getX() - currentPose.getX();
    double deltaY = goalPose.getY() - currentPose.getY();
    this.log("Move Dx", deltaX);
    this.log("Move Dy", deltaY);
    double deltaPose = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    // if (deltaPose > AutoAlignConstants.kMaxDistStillGo) {
    //   drive(0, 0, 0, false, false);
    //   return;
    // }
    double xVelocity = AutoAlignConstants.kPathFollowingP * deltaX;
    double yVelocity = AutoAlignConstants.kPathFollowingP * deltaY;
    double maxSpeed = AutoAlignConstants.kPathFollowingP * deltaPose;
    if (maxSpeed > 1.0) {
      xVelocity /= maxSpeed;
      yVelocity /= maxSpeed;
    }
    double deltaAngle = getAngleToGoal(goalPose.getRotation());
    this.log("Move dTheta", deltaAngle);
    double angularVelocity = deltaAngle * AutoAlignConstants.kPathFollowingAngularP;
    angularVelocity = MathUtil.clamp(angularVelocity, -1.0, 1.0);
    drive(xVelocity, yVelocity, angularVelocity, RelativeTo.FIELD_RELATIVE, true);
  }

  /**
   * Drives to a certain distance from a reference point.
   *
   * @param refPoint The point to be referenced to drive to a distance from.
   * @param goalDist The goal distance from the reference point.
   * @param directionOffset The offset for the direction to face towards the reference point (0
   *     would mean facing the point) in radians.
   */
  public void driveToDistFromPoint(Pose2d refPoint, double goalDist, double directionOffset) {
    Pose2d currentPose = getPose();
    double deltaXRef = refPoint.getX() - currentPose.getX();
    double deltaYRef = refPoint.getY() - currentPose.getY();
    double deltaPoseRef = Math.sqrt(Math.pow(deltaXRef, 2) + Math.pow(deltaYRef, 2));
    // this.log("Dist From Point", deltaPoseRef);
    this.log("Dist to Point", deltaPoseRef - goalDist);
    double deltaX = deltaXRef;
    double deltaY = deltaYRef;
    if (deltaPoseRef > goalDist) {
      deltaX *= (deltaPoseRef - goalDist) / deltaPoseRef;
      deltaY *= (deltaPoseRef - goalDist) / deltaPoseRef;
    } else {
      deltaX *= -(goalDist - deltaPoseRef) / goalDist;
      deltaY *= -(goalDist - deltaPoseRef) / goalDist;
    }
    this.log("Delta x (dist)", deltaX);
    this.log("Delta y (dist)", deltaY);
    double xVelocity = AutoAlignConstants.kPathFollowingP * deltaX;
    double yVelocity = AutoAlignConstants.kPathFollowingP * deltaY;
    double maxSpeed = AutoAlignConstants.kPathFollowingP * Math.abs(deltaPoseRef - goalDist);
    if (maxSpeed > 1.0) {
      xVelocity /= maxSpeed;
      yVelocity /= maxSpeed;
    }
    double deltaAngle = getAngleToPoint(refPoint);
    this.log("Delta angle (dist)", deltaAngle);
    double angleGoal =
        (deltaAngle + directionOffset)
            % (2
                * Math.PI); // (Math.atan2(deltaYRef, deltaXRef) + directionOffset) % (2 * Math.PI);
    double deltaTheta = getAngleToGoal(angleGoal);
    this.log("Delta theta (dist)", deltaTheta);
    double angularVelocity = deltaTheta * AutoAlignConstants.kPathFollowingAngularP;
    angularVelocity = MathUtil.clamp(angularVelocity, -1.0, 1.0);
    drive(xVelocity, yVelocity, angularVelocity, RelativeTo.FIELD_RELATIVE, true);
  }

  public Command driveToPoseCommand(Pose2d bluePose) {
    return this.run(
            () -> {
              driveToGoal(AllianceUtil.getPoseForAlliance(bluePose));
            })
        .until(() -> isAtGoal(AllianceUtil.getPoseForAlliance(bluePose)))
        .andThen(stopCommand());
  }

  public Command driveToSpeakerShootPoseCommand() {
    return driveToPoseCommand(AutoAlignConstants.kBlueSpeakerShootPose);
  }

  public Command driveToAmpPoseCommand() {
    return driveToPoseCommand(AutoAlignConstants.kBlueAmpShootPose);
  }

  public boolean isSpeakerAligned() {
    return isAtDistFromPoint(
            AllianceUtil.getPoseForAlliance(AutoAlignConstants.kBlueSpeakerPose),
            AutoAlignConstants.kShootDistFromSpeaker)
        && getAngleToGoal(
                new Rotation2d(
                    getAngleToPoint(
                            AllianceUtil.getPoseForAlliance(AutoAlignConstants.kBlueSpeakerPose))
                        + Math.PI))
            < AutoAlignConstants.kAtRotationGoalTolerance;
  }

  public Command driveToShootInSpeakerCommand() {
    return this.run(
            () -> {
              driveToDistFromPoint(
                  AllianceUtil.getPoseForAlliance(AutoAlignConstants.kBlueSpeakerPose),
                  AutoAlignConstants.kShootDistFromSpeaker,
                  Math.PI); // So the back (with the shooter) is facing the speaker.
            })
        .until(
            () -> {
              boolean aligned =
                  isAtDistFromPoint(
                          AllianceUtil.getPoseForAlliance(AutoAlignConstants.kBlueSpeakerPose),
                          AutoAlignConstants.kShootDistFromSpeaker)
                      && getAngleToGoal(
                              new Rotation2d(
                                  getAngleToPoint(
                                          AllianceUtil.getPoseForAlliance(
                                              AutoAlignConstants.kBlueSpeakerPose))
                                      + Math.PI))
                          < AutoAlignConstants.kAtRotationGoalTolerance;
              return aligned;
            })
        .andThen(stopCommand());
  }

  private Command twoConditionCommand(
      BooleanSupplier ifCondition,
      Command ifCommand,
      BooleanSupplier elseIfCondition,
      Command elseIfCommand,
      Command elseCommand) {
    return new ConditionalCommand(
        ifCommand,
        new ConditionalCommand(elseIfCommand, elseCommand, elseIfCondition),
        ifCondition);
  }

  public Command autoDriveToSpeakerShoot() {
    return twoConditionCommand(
        () -> {
          double angle =
              getAngleFromPointPositive(
                  AllianceUtil.getPoseForAlliance(AutoAlignConstants.kBlueSpeakerPose));
          return AutoAlignConstants.kMaxAngleSpeakerShootOffset < angle
              && angle < Math.PI - AutoAlignConstants.kMaxAngleSpeakerShootOffset;
        },
        driveToPoseCommand(AutoAlignConstants.kBlueMaxSpeakerShootPose),
        () -> {
          double angle =
              getAngleFromPointPositive(
                  AllianceUtil.getPoseForAlliance(AutoAlignConstants.kBlueSpeakerPose));
          return Math.PI + AutoAlignConstants.kMaxAngleSpeakerShootOffset < angle
              && angle < 2 * Math.PI - AutoAlignConstants.kMaxAngleSpeakerShootOffset;
        },
        driveToPoseCommand(AutoAlignConstants.kBlueMinSpeakerShootPose),
        driveToShootInSpeakerCommand());
  }

  public Command pickUpNotePoseCommand(AllianceNoteLocation noteLocation) {
    return this.run(
            () -> {
              driveToDistFromPoint(noteLocation.getPose(), AutoAlignConstants.kPickUpNoteDist, 0.0);
            })
        .until(
            () -> {
              this.log(
                  "Angle to Point",
                  getAngleToPoint(AllianceUtil.getPoseForAlliance(noteLocation.getPose())));
              this.log(
                  "Angle to Goal",
                  getAngleToGoal(
                      new Rotation2d(
                          getAngleToPoint(
                              AllianceUtil.getPoseForAlliance(noteLocation.getPose())))));
              return isAtDistFromPoint(
                      AllianceUtil.getPoseForAlliance(noteLocation.getPose()),
                      AutoAlignConstants.kPickUpNoteDist)
                  && Math.abs(
                          getAngleToPoint(AllianceUtil.getPoseForAlliance(noteLocation.getPose()))
                              - (getPose().getRotation().getRadians()))
                      < AutoAlignConstants.kAtRotationGoalTolerance;
            });
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
