package lib;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;

public class Utils {
    public static final double EPSILON = 1e-9;

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    public static boolean epsilonEquals(double a, double b, double maxError) {
        return Math.abs(a - b) <= maxError;
    }

    public static boolean speedsEpsilonEquals(ChassisSpeeds speeds) {
        return epsilonEquals(speeds.vxMetersPerSecond, 0)
                && epsilonEquals(speeds.vyMetersPerSecond, 0)
                && epsilonEquals(speeds.omegaRadiansPerSecond, 0);
    }

    public static Pose3d pose2dToPose3d(Pose2d pose) {
        return new Pose3d(
                pose.getX(), pose.getY(), 0, new Rotation3d(0, 0, pose.getRotation().getRadians()));
    }

    /**
     * Averages ambiguity of estimated poses using a harmonic average. Can be from different targets
     * in vision module, or between module.
     *
     * @param ambiguities the ambiguities to average.
     * @return the average of the ambiguities.
     */
    public static double averageAmbiguity(List<Double> ambiguities) {
        return 1.0 / ambiguities.stream().map((num) -> 1.0 / num).reduce(0.0, Double::sum);
    }

    public static double normalize(double angle) {
        while (angle < 0) {
            angle += 2 * Math.PI;
        }
        return angle % (2 * Math.PI);
    }

    public static Rotation2d normalize(Rotation2d angle) {
        return Rotation2d.fromRadians(normalize(angle.getRadians()));
    }
}
