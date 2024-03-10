package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.DriveConstants;
import frc.robot.sensors.AprilTagCamera;
import monologue.Logged;

public class SwerveOdometry{
    private Pose2d pose=new Pose2d();
    private int dev;
    private SwerveDrivePoseEstimator poseEstimator;
    private Supplier<SwerveModulePosition[]> positions;
    private Supplier<SwerveModuleState[]> states;
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private Pose2d simPose=new Pose2d();
    boolean isReal=true;
    private ArrayList<AprilTagCamera> cameras=new ArrayList<>();
    public SwerveOdometry(SwerveDriveKinematics kinematics,
            Supplier<SwerveModulePosition[]> positions, Supplier<SwerveModuleState[]> states,String gyroName) {
        this.poseEstimator =new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            gyro.getRotation2d(),
            positions.get(),
            new Pose2d());
        this.positions = positions;
        this.states = states;
        this.dev=SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    }
    public SwerveOdometry addCamera(AprilTagCamera camera){
        cameras.add(camera);
        return this;
    }
    public void setIsReal(boolean real){
        isReal=real;
    }
    public void periodic(){
        poseEstimator.update(
        gyro.getRotation2d(),
        positions.get());
        for(AprilTagCamera camera:cameras){
            camera.addVisionMeasurementToEstimator(poseEstimator);
        }
    }
    public void simulationPeriodic(){
    SimDouble gyroSimAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    gyroSimAngle.set(-getPose().getRotation().getDegrees());

    double timeDelta = 0.020; // standard loop time is 20 ms
    ChassisSpeeds speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(states.get());
    simPose =
        simPose.exp(
            new Twist2d(
                speeds.vxMetersPerSecond * timeDelta,
                speeds.vyMetersPerSecond * timeDelta,
                speeds.omegaRadiansPerSecond * timeDelta));
    }
    public void resetGyro(){
        gyro.zeroYaw();
    }
    public Pose2d getPose(){
        if(isReal)return pose;
        return simPose;
    }
    public Rotation2d gyroAngle(){
        return gyro.getRotation2d();
    }
    public AHRS getGryo(){
        return gyro;
    }
}
