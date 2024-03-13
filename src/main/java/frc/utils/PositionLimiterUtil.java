package frc.utils;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;

public class PositionLimiterUtil {
    private static Supplier<Pose2d> currentPose = () -> new Pose2d();

    private static boolean point1Configured = false;
    private static boolean point2Configured = false;

    private static double x1;
    private static double y1;

    private static double x2;
    private static double y2;

    public static void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
        currentPose = poseSupplier;
    } 

    public static boolean isConfigured() {
        return point1Configured && point2Configured;
    }

    public static void configurePoint1() {
        x1 = currentPose.get().getX();
        y1 = currentPose.get().getY();
        point1Configured = true;
    }

    public static void configurePoint2() {
        x2 = currentPose.get().getX();
        y2 = currentPose.get().getY();
        point2Configured = true;
    }

    public static boolean xVelocityNotAllowed(double desiredVelocity) {
        double currentX = currentPose.get().getX();
        if (desiredVelocity > 0.0) {
            return currentX > x1 && currentX > x2;
        } else if (desiredVelocity < 0.0) {
            return currentX < x1 && currentX < x2;
        } else {
            return true;
        }
    }

    public static boolean yVelocityNotAllowed(double desiredVelocity) {
        double currentY = currentPose.get().getY();
        if (desiredVelocity > 0.0) {
            return currentY > y1 && currentY > y2;
        } else if (desiredVelocity < 0.0) {
            return currentY < y1 && currentY < y2;
        } else {
            return true;
        }
    }
}
