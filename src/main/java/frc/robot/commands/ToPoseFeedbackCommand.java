package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.SwerveUtils;

public class ToPoseFeedbackCommand extends Command {
  DriveSubsystem robotDrive;
  private final Pose2d goalPose;
  private final double halfStartMetersFromGoal;
  private final double startRadiansFromGoal;
  private final double metersOffToStop;
  private final double maxPercentOutput;

  /**
   * Constructs a command to go to a pose with feedback given measurements of pose in imperial, mostly for testing purposes
   * @param robotDrive The robot drive to use
   * @param xFeet X position goal
   * @param yFeet Y position goal
   * @param rotation Rotation goal of position, which is set to current rotation if null
   * @param inchesOff The maximum number of inches the robot can be away from the goal position to stop
   * @param maxPercentOutput The maximum percent output to use
   */
  public ToPoseFeedbackCommand(
      DriveSubsystem robotDrive,
      double xFeet,
      double yFeet,
      Rotation2d rotation,
      double inchesOff,
      double maxPercentOutput) {
    this.robotDrive = robotDrive;
    goalPose = new Pose2d(Units.feetToMeters(xFeet), Units.feetToMeters(yFeet), rotation == null ? robotDrive.getPose().getRotation() : rotation);
    halfStartMetersFromGoal = metersFromGoal() / 2;
    startRadiansFromGoal = radiansFromGoal();
    metersOffToStop = Units.inchesToMeters(inchesOff);
    this.maxPercentOutput = maxPercentOutput;
  }

  /**
   * Constructs a command to go to a pose with feedback given a Pose2d (this can be used with Pose2ds that are known to be at certain locations)
   * @param robotDrive The robot drive to use
   * @param goalPose The goal position
   * @param centimetersOffToStop The maximum number of centimeters the robot can be away from the goal position to stop
   * @param maxPercentOutput The maximum percent output to use
   */
  public ToPoseFeedbackCommand(
      DriveSubsystem robotDrive, Pose2d goalPose, double centimetersOffToStop, double maxPercentOutput) {
    this.robotDrive = robotDrive;
    this.goalPose = goalPose;
    halfStartMetersFromGoal = metersFromGoal() / 2;
    startRadiansFromGoal = radiansFromGoal();
    this.metersOffToStop = centimetersOffToStop / 100;
    this.maxPercentOutput = maxPercentOutput;
  }

  /**
   * @return The meters remaining to go from current pose to goal in the x direction.
   */
  private double xMetersFromGoal() {
    return goalPose.getX() - robotDrive.getPose().getX();
  }

  /**
   * @return The meters remaining to go from current pose to goal in the y direction.
   */
  private double yMetersFromGoal() {
    return goalPose.getY() - robotDrive.getPose().getY();
  }

  /**
   * @return The meters remaining to go from current pose to goal (hypotenuse).
   */
  private double metersFromGoal() {
    return Math.sqrt(Math.pow(xMetersFromGoal(), 2) + Math.pow(yMetersFromGoal(), 2));
  }

  /**
   * @return The radians remaining to get to the goal
   */
  private double radiansFromGoal() {
    return SwerveUtils.WrapAngle(goalPose.getRotation().getRadians())
        - SwerveUtils.WrapAngle(robotDrive.getPose().getRotation().getRadians());
  }

  /**
   * @return Angle between x side and hypotenuse in radians.
   */
  private double radiansComparedToGoal() {
    return Math.atan(yMetersFromGoal() / xMetersFromGoal());
  }

  /**
   * @return The speed the robot should go linearly towards the target pose.
   */
  private double calculateRawSpeed() {
    return Math.abs(metersFromGoal()) > Math.abs(halfStartMetersFromGoal)
        ? maxPercentOutput
        : maxPercentOutput * (metersFromGoal() / Math.abs(halfStartMetersFromGoal));
  }

  /**
   * @return Calculated x speed (meters / s)
   */
  private double calculateXSpeed() {
    return Math.signum(xMetersFromGoal()) * calculateRawSpeed() * Math.cos(radiansComparedToGoal());
  }

  /**
   * @return Calculated y speed (meters / s)
   */
  private double calculateYSpeed() {
    return Math.signum(yMetersFromGoal()) * calculateRawSpeed() * Math.sin(radiansComparedToGoal());
  }

  /**
   * @return Calculated rotation speed (radians / s)
   */
  private double calculateRotationSpeed() {
    return Math.abs(radiansFromGoal()) > Math.abs(0.25 * startRadiansFromGoal)
        ? Math.signum(radiansFromGoal()) * maxPercentOutput
        : Math.signum(radiansFromGoal()) * maxPercentOutput * Math.abs(radiansFromGoal() / (0.25 * startRadiansFromGoal));
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
    return Math.abs(metersFromGoal()) <= metersOffToStop;
  }
}
