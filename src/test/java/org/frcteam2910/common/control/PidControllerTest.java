package org.frcteam2910.common.control;

import org.frcteam2910.common.math.MathUtils;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class PidControllerTest {
    private static final double TEST_DT = 5.0e-3;

    @Test
    void setOutputRangeTest() {
        PidController pidController = new PidController(new PidConstants(1.0, 0.0, 0.0));

        pidController.setSetpoint(5.0);
        assertEquals(5.0, pidController.calculate(0.0, TEST_DT), MathUtils.EPSILON);

        pidController.setOutputRange(-1.0, 1.0);
        assertEquals(1.0, pidController.calculate(0.0, TEST_DT), MathUtils.EPSILON);
        assertEquals(0.5, pidController.calculate(4.5, TEST_DT), MathUtils.EPSILON);
        assertEquals(-1.0, pidController.calculate(10.0, TEST_DT), MathUtils.EPSILON);
        assertEquals(-0.5, pidController.calculate(5.5, TEST_DT), MathUtils.EPSILON);
    }

    @Test
    void setOutputRangeVerifyArgumentsTest() {
        PidController pidController = new PidController(new PidConstants(0.0, 0.0, 0.0));

        assertThrows(IllegalArgumentException.class, () -> pidController.setOutputRange(1.0, -1.0));
    }
}
