package tests;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Assertions;
import lib.math.differential.BooleanTrigger;

public class TestBooleanTrigger {

    @Test
    public void testBooleanTrigger() {
        BooleanTrigger trigger = new BooleanTrigger();

        trigger.update(true);
        trigger.update(false);

        Assertions.assertTrue(
                trigger.released()
        );
        Assertions.assertFalse(
                trigger.triggered()
        );

        trigger.update(true);
        Assertions.assertTrue(
                trigger.triggered()
        );
        Assertions.assertFalse(
                trigger.released()
        );
    }
}
