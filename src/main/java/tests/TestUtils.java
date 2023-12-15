package tests;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import lib.Utils;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class TestUtils {

    @Test
    public void testDeadband() {
        Assertions.assertEquals(
                0,
                Utils.deadband(0.5, 0.5),
                Utils.EPSILON
        );
        Assertions.assertEquals(
                0,
                Utils.deadband(0.25, 0.5),
                Utils.EPSILON
        );
        Assertions.assertEquals(
                0.5,
                Utils.deadband(0.75, 0.5),
                Utils.EPSILON
        );

        Assertions.assertEquals(
                0,
                Utils.deadband(-0.5, 0.5),
                Utils.EPSILON
        );
        Assertions.assertEquals(
                0,
                Utils.deadband(-0.25, 0.5),
                Utils.EPSILON
        );
        Assertions.assertEquals(
                -0.5,
                Utils.deadband(-0.75, 0.5),
                Utils.EPSILON
        );
    }

    @Test
    public void testEpsilonEqualsDefaultEpsilon() {
        Assertions.assertTrue(
                Utils.epsilonEquals(1, 1)
        );
        Assertions.assertTrue(
                Utils.epsilonEquals(-1, -1)
        );
        Assertions.assertTrue(
                Utils.epsilonEquals(0, 0)
        );

        Assertions.assertTrue(
                Utils.epsilonEquals(1 + Utils.EPSILON / 2, 1)
        );
        Assertions.assertTrue(
                Utils.epsilonEquals(1, 1 + Utils.EPSILON / 2)
        );
        Assertions.assertTrue(
                Utils.epsilonEquals(1, 1 - Utils.EPSILON / 2)
        );
        Assertions.assertTrue(
                Utils.epsilonEquals(1 - Utils.EPSILON / 2, 1)
        );

        Assertions.assertTrue(
                Utils.epsilonEquals(-1 + Utils.EPSILON / 2, -1)
        );
        Assertions.assertTrue(
                Utils.epsilonEquals(-1, -1 + Utils.EPSILON / 2)
        );
        Assertions.assertTrue(
                Utils.epsilonEquals(-1, -1 - Utils.EPSILON / 2)
        );
        Assertions.assertTrue(
                Utils.epsilonEquals(-1 - Utils.EPSILON / 2, -1)
        );
    }

    @Test
    public void testEpsilonEqualsCustomEpsilon() {
        Assertions.assertTrue(
                Utils.epsilonEquals(1, 1.5, 0.5)
        );
        Assertions.assertTrue(
                Utils.epsilonEquals(-1, -1.5, 0.5)
        );
        Assertions.assertTrue(
                Utils.epsilonEquals(1.5, 1, 0.5)
        );
        Assertions.assertTrue(
                Utils.epsilonEquals(-1.5, -1, 0.5)
        );
    }

    @Test
    public void testPose2dConversions() {
        double x = 1, y = 1, rot = 1;
        Pose2d poseObject = new Pose2d(1, 1, new Rotation2d(1));
        double[] poseArray = new double[]{x, y, rot};

        Assertions.assertArrayEquals(
                poseArray,
                Utils.pose2dToArray(poseObject),
                Utils.EPSILON
        );
        Assertions.assertEquals(
                poseObject.toString(),
                Utils.arrayToPose2d(poseArray).toString()
        );
    }

    @Test
    public void testChassisSpeedsConversions() {
        double x = 1, y = 1, omega = 1;
        double[] chassisSpeedsArray = new double[]{x, y, omega};
        ChassisSpeeds chassisSpeedsObject = new ChassisSpeeds(x, y, omega);

        Assertions.assertArrayEquals(
                chassisSpeedsArray,
                Utils.chassisSpeedsToArray(chassisSpeedsObject),
                Utils.EPSILON
        );
        Assertions.assertEquals(
                Utils.arrayToChassisSpeeds(chassisSpeedsArray).toString(),
                chassisSpeedsObject.toString()
        );
    }

    @Test
    public void testSwerveModuleConversions() {
        double vel1 = 1, angle1 = 1, vel2 = 2, angle2 = 0.5;
        double[] swerveModuleStatesArray = new double[]{angle1, vel1, angle2, vel2};
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[]{
                new SwerveModuleState(vel1, new Rotation2d(angle1)),
                new SwerveModuleState(vel2, new Rotation2d(angle2))
        };

        SwerveModuleState[] convertedSwerveModuleStates =
                Utils.arrayToSwerveModuleStates(swerveModuleStatesArray);

        Assertions.assertEquals(
                swerveModuleStates.length,
                convertedSwerveModuleStates.length
        );
        for (int i = 0; i < convertedSwerveModuleStates.length; i++) {
            Assertions.assertEquals(
                    "Module " + (i + 1),
                    swerveModuleStates[i].toString(),
                    convertedSwerveModuleStates[i].toString()
            );
        }

        Assertions.assertArrayEquals(
                swerveModuleStatesArray,
                Utils.swerveModuleStatesToArray(swerveModuleStates),
                Utils.EPSILON
        );
    }

    @Test
    public void testSpeedsEpsilonEquals() {
        double x = Utils.EPSILON / 2, y = Utils.EPSILON / 2, omega = Utils.EPSILON / 2;
        ChassisSpeeds speeds = new ChassisSpeeds(x, y, omega);
        ChassisSpeeds negativeSpeeds = new ChassisSpeeds(-x, -y, -omega);

        Assertions.assertTrue(
                Utils.speedsEpsilonEquals(speeds)
        );
        Assertions.assertTrue(
                Utils.speedsEpsilonEquals(negativeSpeeds)
        );
        Assertions.assertTrue(
                Utils.speedsEpsilonEquals(new ChassisSpeeds())
        );
    }
}
