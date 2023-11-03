package frc.robot.common.tests.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.common.utils.Utils;
import org.junit.Assert;
import org.junit.Test;

public class TestUtils {

    @Test
    public void testDeadband() {
        Assert.assertEquals(
                0,
                Utils.deadband(0.5, 0.5),
                Utils.EPSILON
        );
        Assert.assertEquals(
                0,
                Utils.deadband(0.25, 0.5),
                Utils.EPSILON
        );
        Assert.assertEquals(
                0.5,
                Utils.deadband(0.75, 0.5),
                Utils.EPSILON
        );

        Assert.assertEquals(
                0,
                Utils.deadband(-0.5, 0.5),
                Utils.EPSILON
        );
        Assert.assertEquals(
                0,
                Utils.deadband(-0.25, 0.5),
                Utils.EPSILON
        );
        Assert.assertEquals(
                -0.5,
                Utils.deadband(-0.75, 0.5),
                Utils.EPSILON
        );
    }

    @Test
    public void testEpsilonEqualsDefaultEpsilon() {
        Assert.assertTrue(
                Utils.epsilonEquals(1, 1)
        );
        Assert.assertTrue(
                Utils.epsilonEquals(-1, -1)
        );
        Assert.assertTrue(
                Utils.epsilonEquals(0, 0)
        );

        Assert.assertTrue(
                Utils.epsilonEquals(1 + Utils.EPSILON / 2, 1)
        );
        Assert.assertTrue(
                Utils.epsilonEquals(1, 1 + Utils.EPSILON / 2)
        );
        Assert.assertTrue(
                Utils.epsilonEquals(1, 1 - Utils.EPSILON / 2)
        );
        Assert.assertTrue(
                Utils.epsilonEquals(1 - Utils.EPSILON / 2, 1)
        );

        Assert.assertTrue(
                Utils.epsilonEquals(-1 + Utils.EPSILON / 2, -1)
        );
        Assert.assertTrue(
                Utils.epsilonEquals(-1, -1 + Utils.EPSILON / 2)
        );
        Assert.assertTrue(
                Utils.epsilonEquals(-1, -1 - Utils.EPSILON / 2)
        );
        Assert.assertTrue(
                Utils.epsilonEquals(-1 - Utils.EPSILON / 2, -1)
        );
    }

    @Test
    public void testEpsilonEqualsCustomEpsilon() {
        Assert.assertTrue(
                Utils.epsilonEquals(1, 1.5, 0.5)
        );
        Assert.assertTrue(
                Utils.epsilonEquals(-1, -1.5, 0.5)
        );
        Assert.assertTrue(
                Utils.epsilonEquals(1.5, 1, 0.5)
        );
        Assert.assertTrue(
                Utils.epsilonEquals(-1.5, -1, 0.5)
        );
    }

    @Test
    public void testPose2dConversions() {
        double x = 1, y = 1, rot = 1;
        Pose2d poseObject = new Pose2d(1, 1, new Rotation2d(1));
        double[] poseArray = new double[]{x, y, rot};

        Assert.assertArrayEquals(
                poseArray,
                Utils.pose2dToArray(poseObject),
                Utils.EPSILON
        );
        Assert.assertEquals(
                poseObject.toString(),
                Utils.arrayToPose2d(poseArray).toString()
        );
    }

    @Test
    public void testChassisSpeedsConversions() {
        double x = 1, y = 1, omega = 1;
        double[] chassisSpeedsArray = new double[]{x, y, omega};
        ChassisSpeeds chassisSpeedsObject = new ChassisSpeeds(x, y, omega);

        Assert.assertArrayEquals(
                chassisSpeedsArray,
                Utils.chassisSpeedsToArray(chassisSpeedsObject),
                Utils.EPSILON
        );
        Assert.assertEquals(
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

        Assert.assertEquals(
                swerveModuleStates.length,
                convertedSwerveModuleStates.length
        );
        for (int i = 0; i < convertedSwerveModuleStates.length; i++) {
            Assert.assertEquals(
                    "Module " + (i + 1),
                    swerveModuleStates[i].toString(),
                    convertedSwerveModuleStates[i].toString()
            );
        }

        Assert.assertArrayEquals(
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

        Assert.assertTrue(
                Utils.speedsEpsilonEquals(speeds)
        );
        Assert.assertTrue(
                Utils.speedsEpsilonEquals(negativeSpeeds)
        );
        Assert.assertTrue(
                Utils.speedsEpsilonEquals(new ChassisSpeeds())
        );
    }
}
