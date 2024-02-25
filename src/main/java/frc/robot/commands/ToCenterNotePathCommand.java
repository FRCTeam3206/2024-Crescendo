package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.CenterNoteLocation;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.AllianceUtil;

/** Constructs a command that goes to a center note. */
public class ToCenterNotePathCommand extends Command {
  DriveSubsystem robotDrive;
  private CenterNoteLocation goalNote;
  private boolean takeHighPath;

  public ToCenterNotePathCommand(
      DriveSubsystem robotDrive, CenterNoteLocation goalNote, boolean takeHighPath) {
    this.robotDrive = robotDrive;
    this.goalNote = goalNote;
    this.takeHighPath = takeHighPath;
    addRequirements(robotDrive);
  }

  public void execute() {
    // Math.abs(robotDrive.getPose().getX() - AutoAlignConstants.kFieldLength / 2.0) <
    // (AutoAlignConstants.kFieldLength / 2.0 - 6.5)
    if (!AllianceUtil.getDistToAllianceWallLessThan(AutoAlignConstants.kNearCenterMinX)) {
      robotDrive.driveToDistFromPoint(
          goalNote.getTranslation(), AutoAlignConstants.kAtGoalTolerance, 0);
    } else {
      double currentY = robotDrive.getPose().getY();
      Translation2d waypoint;
      double tolerance = AutoAlignConstants.kAtWaypointDist;
      if (takeHighPath) {
        if (currentY < AutoAlignConstants.kLowWaypoint.getY() - tolerance
            && !AllianceUtil.getDistToAllianceWallLessThan(AutoAlignConstants.kLowWaypoint.getX())) {
          waypoint = AutoAlignConstants.kLowWaypoint;
        } else if (currentY < AutoAlignConstants.kMiddleWaypoint.getY() - tolerance) {
          waypoint = AutoAlignConstants.kMiddleWaypoint;
        } else if (currentY < AutoAlignConstants.kHighWaypoint.getY() - tolerance) {
          waypoint = AutoAlignConstants.kHighWaypoint;
        } else {
          waypoint = AutoAlignConstants.kTopWaypoint;
        }
      } else {
        if (currentY > AutoAlignConstants.kHighWaypoint.getY() - tolerance
            && !AllianceUtil.getDistToAllianceWallLessThan(AutoAlignConstants.kHighWaypoint.getX())) {
          waypoint = AutoAlignConstants.kHighWaypoint;
        } else if (currentY > AutoAlignConstants.kMiddleWaypoint.getY() - tolerance) {
          waypoint = AutoAlignConstants.kMiddleWaypoint;
        } else if (currentY > AutoAlignConstants.kLowWaypoint.getY() - tolerance) {
          waypoint = AutoAlignConstants.kLowWaypoint;
        } else {
          waypoint = AutoAlignConstants.kTopWaypoint;
        }
      }
      robotDrive.driveToWaypoint(waypoint, AutoAlignConstants.kMaxWaypointFollowingSpeed);
    }
  }

  public boolean isFinished() {
    return Math.abs(
            robotDrive.getPose().getTranslation().getDistance(goalNote.getTranslation())
                - AutoAlignConstants.kPickUpNoteDist)
        < AutoAlignConstants.kAtGoalTolerance;
  }
}
