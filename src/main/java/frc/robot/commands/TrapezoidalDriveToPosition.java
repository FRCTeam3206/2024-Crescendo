package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.SwerveUtils;

public class TrapezoidalDriveToPosition extends Command{
    private Pose2d target;
    DriveSubsystem drive;
    double minDeltaPos,minDeltaTheta;
    double kDt=.02;
    private final TrapezoidProfile posProfile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(AutoConstants.kMaxSpeedMetersPerSecond,AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    private final TrapezoidProfile angleProfile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(AutoConstants.kMaxAngularSpeedRadiansPerSecond,AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    public TrapezoidalDriveToPosition(Pose2d target, double minDeltaPos, double minDeltaTheta, DriveSubsystem drive){
        this.target=target;
        this.drive=drive;
        this.minDeltaPos=minDeltaPos;
        this.minDeltaTheta=minDeltaTheta;
        this.addRequirements(drive);
    }
    private double deltaX(){
        return target.getX()-drive.getPose().getX();
    }
    private double deltaY(){
        return target.getY()-drive.getPose().getY();
    }
    double lastPoseVelocity=0;
    double lastAngleVelocity=0;
    public void execute(){
        double distance=target.minus(drive.getPose()).getTranslation().getNorm();
        double dy=target.minus(drive.getPose()).getY();
        double dx=target.minus(drive.getPose()).getX();
        double angleTo=Math.atan(dy/dx);
        SmartDashboard.putNumber("Angle To", angleTo);
        SmartDashboard.putNumber("Distance", distance);
        double speed=posProfile.calculate(kDt, new TrapezoidProfile.State(distance,lastPoseVelocity), goal).velocity;
        lastPoseVelocity=speed;
        SmartDashboard.putNumber("Speed", speed);
        double xSpeed=Math.signum(deltaX())*Math.abs(speed*Math.cos(angleTo));
        SmartDashboard.putNumber("XSpeed", xSpeed);
        double ySpeed=Math.signum(deltaY())*Math.abs(speed*Math.sin(angleTo));
        SmartDashboard.putNumber("YSpeed", ySpeed);
        
        double dt=target.getRotation().getRadians()-drive.getPose().getRotation().getRadians();
        SmartDashboard.putNumber("Des Rotation",target.getRotation().getRadians());
        SmartDashboard.putNumber("Current Rotation",drive.getPose().getRotation().getRadians());
        dt=SwerveUtils.WrapAngle(dt+Math.PI)-Math.PI;//Wrap from -180-180
        SmartDashboard.putNumber("Delta Theta", dt);
        double angularSpeed=-angleProfile.calculate(kDt, new TrapezoidProfile.State(dt,lastAngleVelocity), goal).velocity;
        
        drive.driveSpeed(xSpeed, ySpeed, angularSpeed, true, false);
    }
}
