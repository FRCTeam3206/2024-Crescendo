// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.utils.AllianceUtil;
import java.io.IOException;
import java.util.List;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kTurningDampener = 1.8;

    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 11;
    public static final int kRearLeftTurningCanId = 13;
    public static final int kFrontRightTurningCanId = 12;
    public static final int kRearRightTurningCanId = 14;

    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor =
        (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor =
        ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor =
        (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput =
        kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class ShootakeConstants {
    public static final double kXboxDeadband = 0.05;
    public static final int kTopCANID = 6;
    public static final int kBottomCANID = 7;
    public static final int kFingerPort = 0;
    public static final int kNoteSensorChannel = 0;

    public static final double kIntakeSpeed = 0.85;
    public static final double kSlowIntakeSpeed = 0.2;
    public static final double kOutakeSpeed = -0.2;
    public static final double kAmpSpeed = -.3;
    public static final double kShootakeFreeSpeed = 5000.0;
    public static final double kShootakeLoadSpeedThreshold = 4750.0;
  }

  public static final class ArmConstants {
    public static final double kS = 0.0;
    public static final double kG = 1.55;
    public static final double kV = 0.0;

    public static final double kP = 1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final int kArmCANId = 5;
    public static final double kArmZeroOffset = 0.441;

    public static final double kArmZeroThreshold = Math.PI * .15;

    public static final double kArmAmpAngle = 2.1;
    public static final double kShootAngle = 0.069;
    public static final double kIntakeAngle = 3.434;
    public static final double kSubwooferAngle = 1.380;

    public static final double kAtAngleTolerance = 0.05;
  }

  public static final class ClimberConstants {
    public static final int kLeftClimberCANId = 8;
    public static final int kRightClimberCANId = 9;
  }

  public static enum ArmPostition {
    SHOOT,
    AMP,
    INTAKE
  }

  public static enum RelativeTo {
    ROBOT_RELATIVE,
    DRIVER_RELATIVE,
    FIELD_RELATIVE;
  }

  public static enum AllianceNoteLocation {
    BOTTOM(AutoAlignConstants.kBlueBottomNotePose),
    MIDDLE(AutoAlignConstants.kBlueMiddleNotePose),
    TOP(AutoAlignConstants.kBlueTopNotePose);

    private Pose2d bluePose;

    private AllianceNoteLocation(Pose2d bluePose) {
      this.bluePose = bluePose;
    }

    public Pose2d getPose() {
      return AllianceUtil.getPoseForAlliance(bluePose);
    }
  }

  public static enum CenterNoteLocation {
    TOP(AutoAlignConstants.kCenterNoteT, true),
    TOP_MIDDLE(AutoAlignConstants.kCenterNoteTM, false),
    MIDDLE(AutoAlignConstants.kCenterNoteM, false),
    BOTTOM_MIDDLE(AutoAlignConstants.kCenterNoteBM, false),
    BOTTOM(AutoAlignConstants.kCenterNoteB, true);

    private Translation2d translation;
    private Pose2d blueUpperPickUpPose;
    private Pose2d blueLowerPickUpPose;

    private CenterNoteLocation(Translation2d translation, boolean onlyOnePickUpPose) {
      this.translation = translation;
      if (onlyOnePickUpPose) {
        blueUpperPickUpPose =
            new Pose2d(
                translation.getX() - AutoAlignConstants.kPickUpNoteDist,
                translation.getY(),
                new Rotation2d(0.0));
        blueLowerPickUpPose =
            new Pose2d(
                translation.getX() - AutoAlignConstants.kPickUpNoteDist,
                translation.getY(),
                new Rotation2d(0.0));
      } else {
        blueUpperPickUpPose =
            new Pose2d(
                translation.getX()
                    - AutoAlignConstants.kPickUpNoteDist
                        * Math.cos(AutoAlignConstants.kCenterNotePickUpAngleOffset),
                translation.getY()
                    + AutoAlignConstants.kPickUpNoteDist
                        * Math.sin(AutoAlignConstants.kCenterNotePickUpAngleOffset),
                new Rotation2d(2 * Math.PI - AutoAlignConstants.kCenterNotePickUpAngleOffset));
        blueLowerPickUpPose =
            new Pose2d(
                translation.getX()
                    - AutoAlignConstants.kPickUpNoteDist
                        * Math.cos(AutoAlignConstants.kCenterNotePickUpAngleOffset),
                translation.getY()
                    - AutoAlignConstants.kPickUpNoteDist
                        * Math.sin(AutoAlignConstants.kCenterNotePickUpAngleOffset),
                new Rotation2d(AutoAlignConstants.kCenterNotePickUpAngleOffset));
      }
    }

    public Translation2d getTranslation() {
      return translation;
    }

    public Pose2d getBlueUpperPickUpPose() {
      return blueUpperPickUpPose;
    }

    public Pose2d getBlueLowerPickUpPose() {
      return blueLowerPickUpPose;
    }

    // public Translation2d getPickUpTranslation() {
    //   return new Translation2d(
    //       translation.getX(), translation.getY() - AutoAlignConstants.kPickUpNoteDist);
    // }
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;

    @SuppressWarnings("unused")
    private static final class StandardButtons {
      public static final int x = 1;
      public static final int a = 2;
      public static final int b = 3;
      public static final int y = 4;
      public static final int lb = 5;
      public static final int rb = 6;
      public static final int lt = 7;
      public static final int rt = 8;
      public static final int back = 9;
      public static final int start = 10;
      public static final int leftStickPressed = 11;
      public static final int rightStickPressed = 12;
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kMaxModuleSpeedMetersPerSecond =
        4.803648; // For highest speed of 15.76 feet per second

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class AutoAlignConstants { // Also for driving to pose in general.
    public static final double kFieldLength = Units.inchesToMeters(2.0 * (76.1 + 250.50));
    public static final double kFieldHeight = Units.inchesToMeters(323.00);

    public static final double kAtGoalTolerance = 0.02; // Decide/tune/test
    public static final double kAtRotationGoalTolerance = 0.04; // Decide/tune/test
    public static final double kPathFollowingP = 0.6875; // Tune?
    public static final double kPathFollowingAngularP = 2.0 / Math.PI;
    public static final double kShootDistFromSpeaker = 3.00; // 3.11; // Tune value
    public static final double kShootDistAmp = 0.75; // Find value
    public static final double kPickUpNoteDist = 0.9;
    public static final double kMaxWaypointFollowingSpeed = 3.0; // Meters per second
    public static final double kMaxAngleSpeakerShootOffset = .773;
    public static final double kAtWaypointDist = 0.15;
    public static final double kCenterNotePickUpAngleOffset = Math.PI / 4;
    // Math.PI / 8.0; // Not used yet //.672
    // public static final double kMaxDistStillGo = 4.0; // Decide/tune/test
    // The maximum distance from goal for which the robot should still drive.

    public static final Pose2d kBlueSpeakerPose =
        new Pose2d(Units.inchesToMeters(-1.50), Units.inchesToMeters(218.42), new Rotation2d());

    public static final Pose2d kBlueSpeakerShootPose =
        new Pose2d(
            kBlueSpeakerPose.getX() + kShootDistFromSpeaker,
            kBlueSpeakerPose.getY(),
            kBlueSpeakerPose.getRotation());

    public static final Pose2d kBlueMaxSpeakerShootPose =
        new Pose2d(
            kBlueSpeakerPose.getX() + kShootDistFromSpeaker * Math.cos(kMaxAngleSpeakerShootOffset),
            kBlueSpeakerPose.getY() + kShootDistFromSpeaker * Math.sin(kMaxAngleSpeakerShootOffset),
            new Rotation2d(kMaxAngleSpeakerShootOffset));

    public static final Pose2d kBlueMinSpeakerShootPose =
        new Pose2d(
            kBlueMaxSpeakerShootPose.getX(),
            kBlueSpeakerPose.getY()
                + kShootDistFromSpeaker * Math.sin(2 * Math.PI - kMaxAngleSpeakerShootOffset),
            new Rotation2d(2 * Math.PI - kMaxAngleSpeakerShootOffset));

    public static final Pose2d kBlueAmpPose =
        new Pose2d(Units.inchesToMeters(72.5), Units.inchesToMeters(323.00), new Rotation2d());
    public static final Pose2d kBlueAmpShootPose =
        new Pose2d(
            kBlueAmpPose.getX(), kBlueAmpPose.getY() - kShootDistAmp, new Rotation2d(Math.PI / 2));

    public static final Pose2d kBlueBottomNotePose =
        new Pose2d(Units.inchesToMeters(114.0), Units.inchesToMeters(161.638409), new Rotation2d());
    public static final Pose2d kBlueMiddleNotePose =
        new Pose2d(
            Units.inchesToMeters(114.0), Units.inchesToMeters(161.638409 + 57.0), new Rotation2d());
    public static final Pose2d kBlueTopNotePose =
        new Pose2d(
            Units.inchesToMeters(114.0),
            Units.inchesToMeters(161.638409 + 114.0),
            new Rotation2d());

    private static final double kCenterNoteX = kFieldLength / 2.0;

    /**
     * @param numFromTop Top is 0
     */
    private static final double getCenterNoteY(int numFromTop) {
      return kFieldHeight - Units.inchesToMeters(29.64 + 66.0 * numFromTop);
    }

    public static final Translation2d kCenterNoteT =
        new Translation2d(kCenterNoteX, getCenterNoteY(0));
    public static final Translation2d kCenterNoteTM =
        new Translation2d(kCenterNoteX, getCenterNoteY(1));
    public static final Translation2d kCenterNoteM =
        new Translation2d(kCenterNoteX, getCenterNoteY(2));
    public static final Translation2d kCenterNoteBM =
        new Translation2d(kCenterNoteX, getCenterNoteY(3));
    public static final Translation2d kCenterNoteB =
        new Translation2d(kCenterNoteX, getCenterNoteY(4));

    // Waypoints for going to center notes, in order of lowest to highest (from sim)
    // For blue alliance; must be mirrored
    public static final Translation2d kBottomWaypoint = new Translation2d(5.8, 1.08); // x 6.7
    public static final Translation2d kLowWaypoint = new Translation2d(2.50, 2.5); // y 1.96
    public static final Translation2d kMiddleWaypoint = new Translation2d(2.03, 4.44);
    public static final Translation2d kHighWaypoint = new Translation2d(3.50, 6.34);
    public static final Translation2d kTopWaypoint = new Translation2d(5.8, 6.909); // x 6.7

    public static final double kNearCenterMinX = 5.7 - kAtGoalTolerance;

    // public static final Pose2d kBlueShootPose = new Pose2d(3.110, 5.326, new Rotation2d());
    // public static final Pose2d kRedShootPose =
    //     mapBluePoseToRed(kBlueShootPose); // new Pose2d(13.349, 5.326, new Rotation2d());

  }

  public static final class VisionConstants {
    // TODO Figure out how much to trust state pose estimate versus vision pose estimate (higher
    // number = trust less)
    public static final Matrix<N3, N1> kStateStandardDeviations = VecBuilder.fill(0.5, 0.5, 0.5);
    public static final Matrix<N3, N1> kVisionStandardDeviations = VecBuilder.fill(0.5, 0.5, 0.5);

    // TODO Figure out how much to trust single versus multitag
    public static final Matrix<N3, N1> kSingleTagStandardDeviations = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStandardDeviations = VecBuilder.fill(0.5, 0.5, 1);

    // TODO Add information for camera 1
    public static final String kCameraName1 = "Camera1"; // -0.127287;0.177495
    public static final Transform3d kDistToCamera1 =
        new Transform3d(
            0.155575, -0.152, -0.441758, new Rotation3d(0.0, (Math.PI / 180.0) * -22.0, Math.PI));

    // TODO Add information for camera 2
    public static final String kCameraName2 = "Camera2";
    public static final Transform3d kDistToCamera2 =
        new Transform3d(
            0.155575, 0.152, -0.441758, new Rotation3d(0.0, (Math.PI / 180.0) * -22.0, 0.0));

    public static AprilTagFieldLayout kAprilTagLayout;

    {
      try {
        kAprilTagLayout =
            AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      } catch (IOException e) {
        kAprilTagLayout = new AprilTagFieldLayout(List.of(aprilTagsArray()), 16.541052, 8.211231);
      }
    }

    private static final AprilTag[] aprilTagsArray() {
      /**
       * {@link
       * https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf}
       * Angles in degrees, distances in inches
       */
      double[][] tagPositions = {
        {593.68, 9.68, 53.38, 120},
        {637.21, 34.79, 53.38, 120},
        {652.73, 196.17, 57.13, 180},
        {652.73, 218.42, 57.13, 180},
        {578.77, 323.00, 53.38, 270},
        {72.5, 323.00, 53.38, 270},
        {-1.50, 218.42, 57.13, 0},
        {-1.50, 196.17, 57.13, 0},
        {14.02, 34.79, 53.38, 60},
        {57.54, 9.68, 53.38, 60},
        {468.69, 146.19, 52.00, 300},
        {468.69, 177.10, 52.00, 60},
        {441.74, 161.62, 52.00, 180},
        {209.48, 161.62, 52.00, 0},
        {182.73, 177.10, 52.00, 120},
        {182.73, 146.19, 52.00, 240}
      };
      AprilTag[] aprilTagsToReturn = new AprilTag[16];
      for (int i = 0; i < 16; i++) {
        aprilTagsToReturn[i] =
            new AprilTag(
                i + 1,
                new Pose3d(
                    Units.inchesToMeters(tagPositions[i][0]),
                    Units.inchesToMeters(tagPositions[i][1]),
                    Units.inchesToMeters(tagPositions[i][2]),
                    new Rotation3d(0, 0, Units.degreesToRadians(tagPositions[i][3]))));
      }
      return aprilTagsToReturn;
    }
  }
}
