package utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Utils {
    public static final double EPSILON = 1e-9;

    /**
     * Epsilon equals with standard epsilon.
     */
    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    /**
     * Checks if two doubles are equal within a certain error.
     * @param a The first double.
     * @param b The second double.
     * @param maxError The maximum error.
     * @return Whether the two doubles are equal within the error.
     */
    public static boolean epsilonEquals(double a, double b, double maxError) {
        return Math.abs(a - b) < maxError;
    }

    /**
     * Converts a pose to an array.
     * @param pose The pose to convert.
     * @return The converted pose.
     */
    public static double[] pose2dToArray(Pose2d pose) {
        return new double[]{pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
    }

    /**
     * Converts chassis speeds to an array.
     * @param speeds The chassis speeds to convert.
     * @return The converted chassis speeds.
     */
    public static double[] chassisSpeedsToArray(ChassisSpeeds speeds) {
        return new double[]{speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond};
    }

    /**
     * Converts an array to chassis speeds.
     * @param array The array to convert.
     * @return The converted array.
     */
    public static ChassisSpeeds arrayToChassisSpeeds(double[] array) {
        return new ChassisSpeeds(array[0], array[1], array[2]);
    }

    /**
     * Converts an array of module states to a double array.
     * @param states The module states to convert.
     * @return The converted module states.
     */
    public static double[] swerveModuleStatesToArray(SwerveModuleState[] states) {
        double[] array = new double[states.length * 2];
        for (int i = 0; i < states.length; i++) {
            array[i * 2] = states[i].angle.getRadians();
            array[i * 2 + 1] = states[i].speedMetersPerSecond;
        }
        return array;
    }

    /**
     * Checks whether chassis speeds are epsilon equals to zero.
     * @param speeds The chassis speeds to check.
     * @return Whether the chassis speeds are epsilon equals to zero.
     */
    public static boolean speedsEpsilonEquals(ChassisSpeeds speeds) {
        return epsilonEquals(speeds.vxMetersPerSecond, 0) &&
                epsilonEquals(speeds.vyMetersPerSecond, 0) &&
                epsilonEquals(speeds.omegaRadiansPerSecond, 0);
    }
}