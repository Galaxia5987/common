package lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Utils {
    public static final double EPSILON = 1e-9;

    /**
     * sets the value of the joystick to 0 if the value is less than the threshold
     *
     * @param val       the joystick value
     * @param threshold the threshold value
     * @return 0 if val is less than the threshold else val
     */
    public static double deadband(double val, double threshold) {
        if (Math.abs(val) < threshold)
            return 0;
        return (val - Math.signum(val) * threshold) / (1 - threshold);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    public static boolean epsilonEquals(double a, double b, double maxError) {
        return Math.abs(a - b) <= maxError;
    }

    public static double[] pose2dToArray(Pose2d pose) {
        return new double[]{pose.getX(), pose.getY(), pose.getRotation().getRadians()};
    }

    public static Pose2d arrayToPose2d(double[] array) {
        return new Pose2d(
                array[0],
                array[1],
                new Rotation2d(array[2])
        );
    }

    public static double[] chassisSpeedsToArray(ChassisSpeeds speeds) {
        return new double[]{speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond};
    }

    public static ChassisSpeeds arrayToChassisSpeeds(double[] array) {
        return new ChassisSpeeds(array[0], array[1], array[2]);
    }

    public static double[] swerveModuleStatesToArray(SwerveModuleState[] states) {
        double[] array = new double[states.length * 2];
        for (int i = 0; i < states.length; i++) {
            array[i * 2] = states[i].angle.getRadians();
            array[i * 2 + 1] = states[i].speedMetersPerSecond;
        }
        return array;
    }

    public static SwerveModuleState[] arrayToSwerveModuleStates(double[] states) { //TODO: check
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[states.length / 2];
        for (int i = 0; i < states.length; i += 2) {
            swerveModuleStates[i / 2] = new SwerveModuleState(states[i + 1], new Rotation2d(states[i]));
        }
        return swerveModuleStates;
    }

    public static boolean speedsEpsilonEquals(ChassisSpeeds speeds) {
        return epsilonEquals(speeds.vxMetersPerSecond, 0) &&
                epsilonEquals(speeds.vyMetersPerSecond, 0) &&
                epsilonEquals(speeds.omegaRadiansPerSecond, 0);
    }
    public static double[] pose3dToArray(Pose3d pose){
        return new double[]{
                pose.getX(), pose.getY(), pose.getZ(),
                pose.getRotation().getQuaternion().getW(), pose.getRotation().getQuaternion().getX(), pose.getRotation().getQuaternion().getY(), pose.getRotation().getQuaternion().getZ()};
    }
}