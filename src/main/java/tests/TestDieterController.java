package tests;

import lib.Utils;
import lib.controllers.DieterController;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class TestDieterController {

    @Test
    public void testDieterController() {
        try (DieterController controller = new DieterController(1, 0, 0, 1)) {
            controller.setDieterBand(0.9);
            double val = controller.calculate(0, 1);
            Assertions.assertEquals(
                    2,
                    val,
                    Utils.EPSILON
            );

            controller.setDieterBand(1.1);
            val = controller.calculate(0, 1);
            Assertions.assertEquals(
                    1,
                    val,
                    Utils.EPSILON
            );

            val = controller.calculate(0, 0);
            Assertions.assertEquals(
                    0,
                    val,
                    Utils.EPSILON
            );

            controller.setP(0);
            val = controller.calculate(0, 1.5);
            Assertions.assertEquals(
                    1,
                    val,
                    Utils.EPSILON
            );

            controller.setPIDF(1, 0, 0, 1);
            controller.setDieterBand(0.9);
            val = controller.calculate(0, -1);
            Assertions.assertEquals(
                    -2,
                    val,
                    Utils.EPSILON
            );

            controller.setDieterBand(1.1);
            val = controller.calculate(0, -1);
            Assertions.assertEquals(
                    -1,
                    val,
                    Utils.EPSILON
            );

            val = controller.calculate(0, 0);
            Assertions.assertEquals(
                    0,
                    val,
                    Utils.EPSILON
            );

            controller.setP(Utils.EPSILON / 2);
            val = controller.calculate(0, -1.5);
            Assertions.assertEquals(
                    -1,
                    val,
                    Utils.EPSILON
            );
        }
    }
}
