package org.frcteam2910.common.util;

import org.frcteam2910.common.math.MathUtils;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class DrivetrainFeedforwardConstantsTest {
    @Test
    void calculateFeedforwardTest() {
        DrivetrainFeedforwardConstants constants = new DrivetrainFeedforwardConstants(0.0, 0.0, 1.0);
        assertEquals(
                1.0,
                constants.calculateFeedforward(0.0, 1.0),
                MathUtils.EPSILON,
                "Static constant is not applied correctly");

        constants = new DrivetrainFeedforwardConstants(1.0, 0.0, 0.0);
        assertEquals(
                2.0,
                constants.calculateFeedforward(2.0, 1.0),
                MathUtils.EPSILON,
                "Velocity constant is not applied correctly");

        constants = new DrivetrainFeedforwardConstants(1.0, 0.2, 0.0);
        assertEquals(
                0.2,
                constants.calculateFeedforward(0.0, 1.0),
                MathUtils.EPSILON,
                "Acceleration and velocity constant is not applied correctly");
    }

    @Test
    void getVelocityConstantTest() {
        DrivetrainFeedforwardConstants constants = new DrivetrainFeedforwardConstants(0.0, 0.0, 0.0);
        assertEquals(0.0, constants.getVelocityConstant(), MathUtils.EPSILON, "Incorrect velocity constant returned");

        constants = new DrivetrainFeedforwardConstants(1.0, 0.0, 0.0);
        assertEquals(1.0, constants.getVelocityConstant(), MathUtils.EPSILON, "Incorrect velocity constant returned");
    }

    @Test
    void getAccelerationConstantTest() {
        DrivetrainFeedforwardConstants constants = new DrivetrainFeedforwardConstants(0.0, 0.0, 0.0);
        assertEquals(
                0.0,
                constants.getAccelerationConstant(),
                MathUtils.EPSILON,
                "Incorrect acceleration constant returned");

        constants = new DrivetrainFeedforwardConstants(0.0, 1.0, 0.0);
        assertEquals(
                1.0,
                constants.getAccelerationConstant(),
                MathUtils.EPSILON,
                "Incorrect acceleration constant returned");
    }

    @Test
    void getStaticConstantTest() {
        DrivetrainFeedforwardConstants constants = new DrivetrainFeedforwardConstants(0.0, 0.0, 0.0);
        assertEquals(0.0, constants.getStaticConstant(), MathUtils.EPSILON, "Incorrect static constant returned");

        constants = new DrivetrainFeedforwardConstants(0.0, 0.0, 1.0);
        assertEquals(1.0, constants.getStaticConstant(), MathUtils.EPSILON, "Incorrect static constant returned");
    }
}
