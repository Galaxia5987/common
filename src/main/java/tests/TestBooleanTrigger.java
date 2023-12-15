package tests;

import lib.math.differential.BooleanTrigger;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

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
