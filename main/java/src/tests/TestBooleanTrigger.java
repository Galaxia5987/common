package frc.robot.common.main.java.src.utils;

import frc.robot.common.main.java.src.utils.math.differential.BooleanTrigger;
import org.junit.Assert;
import org.junit.Test;

public class TestBooleanTrigger {

    @Test
    public void testBooleanTrigger() {
        BooleanTrigger trigger = new BooleanTrigger();

        trigger.update(true);
        trigger.update(false);

        Assert.assertTrue(
                trigger.released()
        );
        Assert.assertFalse(
                trigger.triggered()
        );

        trigger.update(true);
        Assert.assertTrue(
                trigger.triggered()
        );
        Assert.assertFalse(
                trigger.released()
        );
    }
}
