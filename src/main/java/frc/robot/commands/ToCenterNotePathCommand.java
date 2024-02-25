package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.CenterNoteLocation;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.AllianceUtil;
import monologue.Logged;

/** Constructs a command that goes to a center note. */
public class ToCenterNotePathCommand extends Command implements Logged{
  DriveSubsystem robotDrive;
  // private CenterNoteLocation goalNote;
  private boolean takeHighPath;
  private Pose2d bluePickUpPose;

  public ToCenterNotePathCommand(
      DriveSubsystem robotDrive, CenterNoteLocation goalNote, boolean takeHighPath) {
    this.robotDrive = robotDrive;
    // this.goalNote = goalNote;
    this.takeHighPath = takeHighPath;
    addRequirements(robotDrive);
    if (takeHighPath) {
      bluePickUpPose = goalNote.getBlueUpperPickUpPose();
    } else {
      bluePickUpPose = goalNote.getBlueLowerPickUpPose();
    }
  }

  public void execute() {
    // Math.abs(robotDrive.getPose().getX() - AutoAlignConstants.kFieldLength / 2.0) <
    // (AutoAlignConstants.kFieldLength / 2.0 - 6.5)
    if (!AllianceUtil.getDistToAllianceWallLessThan(AutoAlignConstants.kNearCenterMinX)) {
      robotDrive.driveToGoal(AllianceUtil.getPoseForAlliance(bluePickUpPose));
      // if (takeHighPath) {
      //   robotDrive.driveToGoal(AllianceUtil.getPoseForAlliance(goalNote.getBlueUpperPickUpPose()));
      // } else {
      //   robotDrive.driveToGoal(AllianceUtil.getPoseForAlliance(goalNote.getBlueLowerPickUpPose()));
      // }
      // robotDrive.driveToDistFromPoint(
      //     goalNote.getTranslation(), AutoAlignConstants.kPickUpNoteDist, 0);
    } else {
      double currentY = robotDrive.getPose().getY();
      Translation2d waypoint;
      double tolerance = AutoAlignConstants.kAtWaypointDist;
      if (takeHighPath) {
        this.log("Path", "High");
        if (currentY < AutoAlignConstants.kLowWaypoint.getY() - tolerance
            && !AllianceUtil.getDistToAllianceWallLessThan(AutoAlignConstants.kLowWaypoint.getX())) {
          waypoint = AllianceUtil.getTranslationForAlliance(AutoAlignConstants.kLowWaypoint);
          robotDrive.driveToWaypoint(waypoint, AutoAlignConstants.kMaxWaypointFollowingSpeed);
          this.log("Waypoint", "Low");
        } else if (currentY < AutoAlignConstants.kMiddleWaypoint.getY() - tolerance) {
          waypoint = AllianceUtil.getTranslationForAlliance(AutoAlignConstants.kMiddleWaypoint);
          robotDrive.driveToWaypoint(waypoint, AutoAlignConstants.kMaxWaypointFollowingSpeed);
          this.log("Waypoint", "Middle");
        } else if (currentY < AutoAlignConstants.kHighWaypoint.getY() - tolerance) {
          waypoint = AllianceUtil.getTranslationForAlliance(AutoAlignConstants.kHighWaypoint);
          robotDrive.driveToWaypoint(waypoint, AutoAlignConstants.kMaxWaypointFollowingSpeed);
          this.log("Waypoint", "High");
        } else {
          waypoint = AllianceUtil.getTranslationForAlliance(AutoAlignConstants.kTopWaypoint);
          robotDrive.driveToWaypoint(waypoint, 0.8, 0.3);
          this.log("Waypoint", "Top");
        }
      } else {
        this.log("Path", "Low");
        if (currentY > AutoAlignConstants.kHighWaypoint.getY() - tolerance
            && !AllianceUtil.getDistToAllianceWallLessThan(AutoAlignConstants.kHighWaypoint.getX())) {
          waypoint = AllianceUtil.getTranslationForAlliance(AutoAlignConstants.kHighWaypoint);
          robotDrive.driveToWaypoint(waypoint, AutoAlignConstants.kMaxWaypointFollowingSpeed);
          this.log("Waypoint", "High");
        } else if (currentY > AutoAlignConstants.kMiddleWaypoint.getY() - tolerance) {
          waypoint = AllianceUtil.getTranslationForAlliance(AutoAlignConstants.kMiddleWaypoint);
          robotDrive.driveToWaypoint(waypoint, AutoAlignConstants.kMaxWaypointFollowingSpeed);
          this.log("Waypoint", "Middle");
        } else if (currentY > AutoAlignConstants.kLowWaypoint.getY() - tolerance) {
          waypoint = AllianceUtil.getTranslationForAlliance(AutoAlignConstants.kLowWaypoint);
          robotDrive.driveToWaypoint(waypoint, AutoAlignConstants.kMaxWaypointFollowingSpeed);
          this.log("Waypoint", "Low");
        } else {
          waypoint = AllianceUtil.getTranslationForAlliance(AutoAlignConstants.kBottomWaypoint);
          robotDrive.driveToWaypoint(waypoint, 0.8, 0.3);
          this.log("Waypoint", "Bottom");
        }
      }
    }
  }

  public boolean isFinished() {
    return robotDrive.getPose().getTranslation().getDistance(bluePickUpPose.getTranslation())
        < AutoAlignConstants.kAtGoalTolerance && 
        robotDrive.getAngleToGoal(bluePickUpPose.getRotation()) < AutoAlignConstants.kAtRotationGoalTolerance; 
  }
}
