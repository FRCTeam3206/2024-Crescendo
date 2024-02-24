package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.RobotContainer.AllianceColor;
import java.util.function.Supplier;

public class AllianceUtil {
  private static AllianceColor alliance = AllianceColor.UNKNOWN;
  private static Supplier<Pose2d> robotPose = () -> new Pose2d();

  public static final Pose2d mapBluePoseToRed(Pose2d bluePose) {
    return new Pose2d(
        AutoAlignConstants.kFieldLength - bluePose.getX(),
        bluePose.getY(),
        new Rotation2d(-(bluePose.getRotation().getRadians() - (Math.PI / 2)) + (Math.PI / 2)));
  }

  public static void setRobot(Supplier<Pose2d> robotPose) {
    AllianceUtil.robotPose = robotPose;
  }

  public static void setAlliance() {
    if (DriverStation.getAlliance().isEmpty()) {
      alliance = AllianceColor.UNKNOWN;
    } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
      alliance = AllianceColor.BLUE;
    } else if (DriverStation.getAlliance().get() == Alliance.Red) {
      alliance = AllianceColor.RED;
    } else {
      alliance = AllianceColor.UNKNOWN;
    }
  }

  public static AllianceColor getAlliance() {
    return alliance;
  }

  public static Pose2d getPoseForAlliance(Pose2d bluePose) {
    Pose2d redPose = mapBluePoseToRed(bluePose);
    if (alliance == AllianceColor.BLUE) {
      return bluePose;
    } else if (alliance == AllianceColor.RED) {
      return redPose;
    } else {
      if (robotPose.get().getTranslation().getDistance(bluePose.getTranslation())
          < robotPose.get().getTranslation().getDistance(redPose.getTranslation())) {
        return bluePose;
      } else {
        return redPose;
      }
    }
  }

  public static Translation2d getTranslationForAlliance(Translation2d blueTranslation) {
    return mapBluePoseToRed(new Pose2d(blueTranslation, new Rotation2d())).getTranslation();
  }
}
