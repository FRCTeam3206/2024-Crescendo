package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer.AllianceColor;
import java.util.function.Supplier;
import monologue.Annotations.Log;

public class AllianceUtil {
  private static AllianceColor alliance = AllianceColor.UNKNOWN;
  private static Supplier<Pose2d> robotPose = () -> new Pose2d();

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

  @Log
  public static AllianceColor getAlliance() {
    return alliance;
  }

  public static Pose2d getPoseForAlliance(Pose2d bluePose, Pose2d redPose) {
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
}
