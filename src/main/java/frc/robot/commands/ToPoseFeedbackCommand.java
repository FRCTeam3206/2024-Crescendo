package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ToPoseFeedbackCommand extends Command {
  DriveSubsystem robotDrive;
  private Pose2d startPose;
  private Pose2d goalPose;
  private double metersOff;
  private double maxOutput;

  public ToPoseFeedbackCommand(DriveSubsystem robotDrive, Pose2d goalPose, double centimetersOff) {
    this.robotDrive = robotDrive;
    startPose = robotDrive.getPose();
    this.goalPose = goalPose;
    metersOff = centimetersOff / 100;
    this.maxOutput = 0.25;
  }

  public ToPoseFeedbackCommand(
      DriveSubsystem robotDrive, Pose2d goalPose, double centimetersOff, double maxOutput) {
    this.robotDrive = robotDrive;
    startPose = robotDrive.getPose();
    this.goalPose = goalPose;
    metersOff = centimetersOff / 100;
    this.maxOutput = maxOutput;
  }

  private double calculateXSpeed() {
    return maxOutput
        * (goalPose.getX() - robotDrive.getPose().getX())
        / Math.abs(goalPose.getX() - startPose.getX());
  }

  private double calculateYSpeed() {
    return maxOutput
        * (goalPose.getY() - robotDrive.getPose().getY())
        / Math.abs(goalPose.getY() - startPose.getY());
  }

  private double calculateRotationSpeed() {
    return maxOutput
        * (goalPose.getRotation().getRadians() - robotDrive.getPose().getRotation().getRadians())
        / Math.abs(goalPose.getRotation().getRadians() - startPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    robotDrive.drive(calculateXSpeed(), calculateYSpeed(), calculateRotationSpeed(), true, true);
  }

  @Override
  public void end(boolean interrupted) {
    robotDrive.drive(0, 0, 0, true, true);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(goalPose.getX() - startPose.getX()) < metersOff
        && Math.abs(goalPose.getY() - startPose.getY()) < metersOff;
  }
}
