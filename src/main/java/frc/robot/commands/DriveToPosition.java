package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.SwerveUtils;

public class DriveToPosition extends Command{
    private Pose2d target;
    DriveSubsystem drive;
    double minDeltaPos,minDeltaTheta;
    public DriveToPosition(Pose2d target, double minDeltaPos, double minDeltaTheta, DriveSubsystem drive){
        this.target=target;
        this.drive=drive;
        this.minDeltaPos=minDeltaPos;
        this.minDeltaTheta=minDeltaTheta;
    }
    Transform2d delta;
    double deltaThetaRadians;
    public void execute(){
        delta=target.minus(drive.getPose());
        double distance=delta.getTranslation().getNorm();
        Rotation2d deltaTheta=delta.getRotation();
        deltaThetaRadians=SwerveUtils.WrapAngle(deltaTheta.getRadians()+Math.PI)-Math.PI;
        double linearSpeed=Math.min(AutoConstants.kMaxSpeedMetersPerSecond/AutoConstants.kMaxDistanceMetersAtFullSpeed*distance,AutoConstants.kMaxSpeedMetersPerSecond);
        double roationalSpeed=Math.min(AutoConstants.kMaxAngularSpeedRadiansPerSecond/AutoConstants.kMaxDeltaThetaAtFullSpeed*deltaThetaRadians,AutoConstants.kMaxAngularSpeedRadiansPerSecond);
        double xDirection=Math.signum(delta.getX());
        double yDirection=Math.signum(delta.getY());
        drive.driveSpeed(xDirection*Math.abs(linearSpeed*deltaTheta.getCos()), yDirection*Math.abs(linearSpeed*deltaTheta.getSin()), roationalSpeed, true,true);
    }
    public boolean isFinished(){
        return Math.abs(delta.getTranslation().getNorm())<minDeltaPos&&deltaThetaRadians<minDeltaTheta;
    }
}
