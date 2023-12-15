package tests;

import lib.Utils;
import lib.math.differential.Integral;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class TestIntegral {

    @Test
    public void testIntegral() {
        Integral integral = new Integral();
        Integral secondIntegral = integral.integrate();

        integral.update(1, 0);
        integral.update(1, 1);
        Assertions.assertEquals(
                1,
                integral.get(),
                Utils.EPSILON
        );
        Assertions.assertEquals(
                0.5,
                secondIntegral.get(),
                Utils.EPSILON
        );

        integral.update(2, 2);
        Assertions.assertEquals(
                2.5,
                integral.get()
        );
        Assertions.assertEquals(
                2.25,
                secondIntegral.get(),
                Utils.EPSILON
        );

        integral.update(1, 3);
        Assertions.assertEquals(
                4,
                integral.get(),
                Utils.EPSILON
        );
        Assertions.assertEquals(
                5.5,
                secondIntegral.get(),
                Utils.EPSILON
        );

        integral.update(0, 4);
        Assertions.assertEquals(
                4.5,
                integral.get(),
                Utils.EPSILON
        );
        Assertions.assertEquals(
                9.75,
                secondIntegral.get(),
                Utils.EPSILON
        );

        integral.update(-4, 5);
        Assertions.assertEquals(
                2.5,
                integral.get(),
                Utils.EPSILON
        );
        Assertions.assertEquals(
                13.25,
                secondIntegral.get(),
                Utils.EPSILON
        );
    }
}
