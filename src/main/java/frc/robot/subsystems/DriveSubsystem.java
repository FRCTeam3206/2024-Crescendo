// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AllianceNoteLocation;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RelativeTo;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.sensors.AprilTagCamera;
import frc.utils.AllianceUtil;
import frc.utils.SwerveUtils;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import monologue.Annotations.Log;
import monologue.Logged;

public class DriveSubsystem extends SubsystemBase implements Logged {
  // Create MAXSwerveModules
  private final MAXSwerveModule frontLeft =
      new MAXSwerveModule(
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule frontRight =
      new MAXSwerveModule(
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule rearLeft =
      new MAXSwerveModule(
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule rearRight =
      new MAXSwerveModule(
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset);

  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotation = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          new Rotation2d(),
          new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition(),
          },
          new Pose2d());

  private final AprilTagCamera poseCamera1 =
      new AprilTagCamera(VisionConstants.kCameraName1, VisionConstants.kDistToCamera1);
  private final AprilTagCamera poseCamera2 =
      new AprilTagCamera(VisionConstants.kCameraName2, VisionConstants.kDistToCamera2);

  SwerveModuleState[] desiredStates =
      new SwerveModuleState[] {
        frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState()
      };

  private Field2d field = new Field2d();
  private SwerveOdometry odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SmartDashboard.putData("Field", field);

    odometry =
        new SwerveOdometry(
                DriveConstants.kDriveKinematics,
                () ->
                    new SwerveModulePosition[] {
                      frontLeft.getPosition(),
                      frontRight.getPosition(),
                      rearLeft.getPosition(),
                      rearRight.getPosition()
                    },
                () -> desiredStates,
                "navX-Sensor[0]")
            .addCamera(poseCamera1)
            .addCamera(poseCamera2);
    odometry.setIsReal(Robot.isReal());
    AllianceUtil.setRobot(odometry::getPose);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.periodic();
    this.log("Robot Pose", odometry.getPose());
    field.setRobotPose(getPose());
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
    return odometry.getPose();
  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(desiredStates);
  }

  /**
   * Updates the odometry (including for sim). Should be called from periodic(). This code comes
   * from Team 2713:
   * https://github.com/FRC2713/Robot2022-v2/blob/main/src/main/java/frc/robot/subsystems/SwerveIO/BabySwerver.java#L126-151
   */

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
      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / currentTranslationMag);
      } else {
        directionSlewRate =
            500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        currentTranslationDir =
            SwerveUtils.StepTowardsCircular(
                currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (currentTranslationMag
            > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          currentTranslationMag = magLimiter.calculate(0.0);
        } else {
          currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      } else {
        currentTranslationDir =
            SwerveUtils.StepTowardsCircular(
                currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(0.0);
      }
      prevTime = currentTime;

      xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
      ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
      currentRotation = rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = currentRotation * DriveConstants.kMaxAngularSpeed;

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
                    xSpeedDelivered, ySpeedDelivered, rotDelivered, odometry.gyroAngle()));
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
    // desiredStates = swerveModuleStates;
    setModuleStates(desiredStates);
    //             SwerveDriveKinematics.desaturateWheelSpeeds(
    //     swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    // frontLeft.setDesiredState(swerveModuleStates[0]);
    // frontRight.setDesiredState(swerveModuleStates[1]);
    // rearLeft.setDesiredState(swerveModuleStates[2]);
    // rearRight.setDesiredState(swerveModuleStates[3]);
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
    drive(0, 0, 0, RelativeTo.FIELD_RELATIVE, false);
  }

  public Command stopCommand() {
    return this.runOnce(() -> stop());
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    System.out.println("Xing");
    setModuleStates(
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        });
    // frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    // frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    // rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    // rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
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
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
    this.desiredStates = desiredStates;
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    odometry.resetGyro();
  }

  public Command zeroHeadingCommand() {
    return this.runOnce(
            () -> {
              zeroHeading();
            })
        .ignoringDisable(true)
        .withName("Reset Gyro");
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  @Log
  public double getHeading() {
    return odometry.gyroAngle().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return odometry.getGryo().getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
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

  public boolean isAtGoal(Pose2d goalPose, double translationTolerance) {
    this.log(
        "Is at Goal",
        getPose().getTranslation().getDistance(goalPose.getTranslation()) < translationTolerance
            && Math.abs(getAngleToGoal(goalPose.getRotation()))
                < AutoAlignConstants.kAtRotationGoalTolerance);
    return getPose().getTranslation().getDistance(goalPose.getTranslation()) < translationTolerance
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
    this.log("Dist to Point", deltaPoseRef);
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

  public Command driveToPoseCommand(Pose2d bluePose, double translationTolerance) {
    return this.run(
            () -> {
              driveToGoal(AllianceUtil.getPoseForAlliance(bluePose));
            })
        .until(() -> isAtGoal(AllianceUtil.getPoseForAlliance(bluePose), translationTolerance))
        .andThen(stopCommand());
  }

  public Command driveToPoseCommand(Pose2d bluePose) {
    return driveToPoseCommand(bluePose, AutoAlignConstants.kAtGoalTolerance);
  }

  public Command driveToSpeakerShootPoseCommand() {
    return driveToPoseCommand(AutoAlignConstants.kBlueSpeakerShootPose);
  }

  public Command driveToAmpSetupPoseCommand() {
    return driveToPoseCommand(AutoAlignConstants.kBlueAmpShootPose);
  }

  public Command scoreToAmpCommand() {
    return driveToAmpSetupPoseCommand()
        .andThen(
            driveCommand(() -> 0, () -> .15, () -> 0, () -> RelativeTo.FIELD_RELATIVE, false)
                .withTimeout(1));
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

    odometry.simulationPeriodic();
  }
}
