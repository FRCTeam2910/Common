package org.frcteam2910.common.util;

import org.frcteam2910.common.math.MathUtils;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class DrivetrainFeedforwardConstantsTest {
    @Test
    public void calculateFeedforwardTest() {
        DrivetrainFeedforwardConstants constants = new DrivetrainFeedforwardConstants(0.0, 0.0, 1.0);
        assertEquals("Static constant is not applied correctly",
                1.0, constants.calculateFeedforward(0.0, 1.0), MathUtils.EPSILON);

        constants = new DrivetrainFeedforwardConstants(1.0, 0.0, 0.0);
        assertEquals("Velocity constant is not applied correctly",
                2.0, constants.calculateFeedforward(2.0, 1.0), MathUtils.EPSILON);

        constants = new DrivetrainFeedforwardConstants(1.0, 0.2, 0.0);
        assertEquals("Acceleration and velocity constant is not applied correctly",
                0.2, constants.calculateFeedforward(0.0, 1.0), MathUtils.EPSILON);
    }

    @Test
    public void getVelocityConstantTest() {
        DrivetrainFeedforwardConstants constants = new DrivetrainFeedforwardConstants(0.0, 0.0, 0.0);
        assertEquals("Incorrect velocity constant returned", 0.0,
                constants.getVelocityConstant(), MathUtils.EPSILON);

        constants = new DrivetrainFeedforwardConstants(1.0, 0.0, 0.0);
        assertEquals("Incorrect velocity constant returned",
                1.0, constants.getVelocityConstant(), MathUtils.EPSILON);
    }

    @Test
    public void getAccelerationConstantTest() {
        DrivetrainFeedforwardConstants constants = new DrivetrainFeedforwardConstants(0.0, 0.0, 0.0);
        assertEquals("Incorrect acceleration constant returned",
                0.0, constants.getAccelerationConstant(), MathUtils.EPSILON);

        constants = new DrivetrainFeedforwardConstants(0.0, 1.0, 0.0);
        assertEquals("Incorrect acceleration constant returned", 1.0,
                constants.getAccelerationConstant(), MathUtils.EPSILON);
    }

    @Test
    public void getStaticConstantTest() {
        DrivetrainFeedforwardConstants constants = new DrivetrainFeedforwardConstants(0.0, 0.0, 0.0);
        assertEquals("Incorrect static constant returned", 0.0,
                constants.getStaticConstant(), MathUtils.EPSILON);

        constants = new DrivetrainFeedforwardConstants(0.0, 0.0, 1.0);
        assertEquals("Incorrect static constant returned", 1.0,
                constants.getStaticConstant(), MathUtils.EPSILON);
    }
}
