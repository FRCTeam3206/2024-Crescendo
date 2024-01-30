package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.SwerveUtils;
import monologue.Annotations.Log;
import monologue.Logged;

public class TrapezoidalDriveToPosition extends Command implements Logged {
  private Pose2d target;
  DriveSubsystem drive;
  double minDeltaPos, minDeltaTheta;
  double kDt = .02;
  @Log.NT private double distance;
  private final TrapezoidProfile posProfile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              AutoConstants.kMaxSpeedMetersPerSecond,
              AutoConstants.kMaxAccelerationMetersPerSecondSquared));
  private final TrapezoidProfile angleProfile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              AutoConstants.kMaxAngularSpeedRadiansPerSecond,
              AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State angleGoal = new TrapezoidProfile.State();

  public TrapezoidalDriveToPosition(
      Pose2d target, double minDeltaPos, double minDeltaTheta, DriveSubsystem drive) {
    this.target = target;
    this.drive = drive;
    this.minDeltaPos = minDeltaPos;
    this.minDeltaTheta = minDeltaTheta;
    this.addRequirements(drive);
  }

  private double deltaX() {
    return target.getX() - drive.getPose().getX();
  }

  private double deltaY() {
    return target.getY() - drive.getPose().getY();
  }

  private double deltaT() {
    double dt = target.getRotation().getRadians() - drive.getPose().getRotation().getRadians();
    SmartDashboard.putNumber("Raw DT", dt);
    dt = SwerveUtils.WrapAngle(dt + Math.PI) - Math.PI;
    return dt;
  }

  double lastPoseVelocity = 0;
  double lastAngleVelocity = 0;

  public void execute() {
    distance = target.minus(drive.getPose()).getTranslation().getNorm();
    double dx = deltaX();
    double dy = deltaY();
    double angleTo = Math.atan(dy / dx);
    double speed =
        posProfile.calculate(kDt, new TrapezoidProfile.State(distance, lastPoseVelocity), goal)
            .velocity;
    lastPoseVelocity = speed;
    double xSpeed = Math.signum(dx) * Math.abs(speed * Math.cos(angleTo));
    double ySpeed = Math.signum(dy) * Math.abs(speed * Math.sin(angleTo));
    if (distance < minDeltaPos) {
      xSpeed = 0;
      ySpeed = 0;
    }
    double dt = deltaT();
    double angularSpeed =
        angleProfile.calculate(kDt, new TrapezoidProfile.State(dt, lastAngleVelocity), angleGoal)
            .velocity;
    lastAngleVelocity = angularSpeed;
    angularSpeed = -angularSpeed;
    drive.driveSpeed(xSpeed, ySpeed, angularSpeed, true, false);
  }

  public void end(boolean interrupted) {
    drive.driveSpeed(0, 0, 0, true, true);
  }

  public boolean isFinished() {
    return Math.sqrt(Math.pow(deltaX(), 2) + Math.pow(deltaY(), 2)) < minDeltaPos
        && Math.abs(deltaT()) < minDeltaTheta;
  }
}
