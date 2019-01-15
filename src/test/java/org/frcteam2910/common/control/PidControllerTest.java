package org.frcteam2910.common.control;

import org.frcteam2910.common.math.MathUtils;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class PidControllerTest {
    private static final double TEST_DT = 5.0e-3;

    @Test
    public void setOutputRangeTest() {
        PidController pidController = new PidController(new PidConstants(1.0, 0.0, 0.0));

        pidController.setSetpoint(5.0);
        assertEquals(5.0, pidController.calculate(0.0, TEST_DT), MathUtils.EPSILON);

        pidController.setOutputRange(-1.0, 1.0);
        assertEquals(1.0, pidController.calculate(0.0, TEST_DT), MathUtils.EPSILON);
        assertEquals(0.5, pidController.calculate(4.5, TEST_DT), MathUtils.EPSILON);
        assertEquals(-1.0, pidController.calculate(10.0, TEST_DT), MathUtils.EPSILON);
        assertEquals(-0.5, pidController.calculate(5.5, TEST_DT), MathUtils.EPSILON);
    }

    @Test(expected = IllegalArgumentException.class)
    public void setOutputRangeVerifyArgumentsTest() {
        PidController pidController = new PidController(new PidConstants(0.0, 0.0, 0.0));

        pidController.setOutputRange(1.0, -1.0);
    }
}
