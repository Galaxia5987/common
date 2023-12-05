package tests;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import utils.Utils;
import utils.math.differential.Derivative;

public class TestDerivative {

    @Test
    public void testDerivative() {
        Derivative derivative = new Derivative();
        Derivative secondDerivative = derivative.differentiate();

        derivative.update(1, 0);
        derivative.update(1, 1);
        Assertions.assertEquals(
                0,
                derivative.get(),
                Utils.EPSILON
        );

        derivative.update(2, 2);
        Assertions.assertEquals(
                1,
                derivative.get(),
                Utils.EPSILON
        );
        Assertions.assertEquals(
                1,
                secondDerivative.get(),
                Utils.EPSILON
        );

        derivative.update(1, 3);
        Assertions.assertEquals(
                -1,
                derivative.get(),
                Utils.EPSILON
        );
        Assertions.assertEquals(
                -2,
                secondDerivative.get(),
                Utils.EPSILON
        );

    }
}
