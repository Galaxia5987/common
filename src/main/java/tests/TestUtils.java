package tests;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lib.Utils;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class TestUtils {

    @Test
    public void testEpsilonEqualsDefaultEpsilon() {
        Assertions.assertTrue(Utils.epsilonEquals(1, 1));
        Assertions.assertTrue(Utils.epsilonEquals(-1, -1));
        Assertions.assertTrue(Utils.epsilonEquals(0, 0));

        Assertions.assertTrue(Utils.epsilonEquals(1 + Utils.EPSILON / 2, 1));
        Assertions.assertTrue(Utils.epsilonEquals(1, 1 + Utils.EPSILON / 2));
        Assertions.assertTrue(Utils.epsilonEquals(1, 1 - Utils.EPSILON / 2));
        Assertions.assertTrue(Utils.epsilonEquals(1 - Utils.EPSILON / 2, 1));

        Assertions.assertTrue(Utils.epsilonEquals(-1 + Utils.EPSILON / 2, -1));
        Assertions.assertTrue(Utils.epsilonEquals(-1, -1 + Utils.EPSILON / 2));
        Assertions.assertTrue(Utils.epsilonEquals(-1, -1 - Utils.EPSILON / 2));
        Assertions.assertTrue(Utils.epsilonEquals(-1 - Utils.EPSILON / 2, -1));
    }

    @Test
    public void testEpsilonEqualsCustomEpsilon() {
        Assertions.assertTrue(Utils.epsilonEquals(1, 1.5, 0.5));
        Assertions.assertTrue(Utils.epsilonEquals(-1, -1.5, 0.5));
        Assertions.assertTrue(Utils.epsilonEquals(1.5, 1, 0.5));
        Assertions.assertTrue(Utils.epsilonEquals(-1.5, -1, 0.5));
    }

    @Test
    public void testSpeedsEpsilonEquals() {
        double x = Utils.EPSILON / 2, y = Utils.EPSILON / 2, omega = Utils.EPSILON / 2;
        ChassisSpeeds speeds = new ChassisSpeeds(x, y, omega);
        ChassisSpeeds negativeSpeeds = new ChassisSpeeds(-x, -y, -omega);

        Assertions.assertTrue(Utils.speedsEpsilonEquals(speeds));
        Assertions.assertTrue(Utils.speedsEpsilonEquals(negativeSpeeds));
        Assertions.assertTrue(Utils.speedsEpsilonEquals(new ChassisSpeeds()));
    }
}
