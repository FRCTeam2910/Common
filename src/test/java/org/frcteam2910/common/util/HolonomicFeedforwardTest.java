package org.frcteam2910.common.util;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class HolonomicFeedforwardTest {
    @Test
    void calculateFeedforwardTest() {
        HolonomicFeedforward feedforward = new HolonomicFeedforward(new DrivetrainFeedforwardConstants(2.0, 0.0, 0.0));
        assertEquals(
                new Translation2d(1.0, 1.0),
                feedforward.calculateFeedforward(new Translation2d(0.5, 0.5), new Translation2d()));

        feedforward = new HolonomicFeedforward(new DrivetrainFeedforwardConstants(0.0, 2.0, 0.0));
        assertEquals(
                new Translation2d(1.0, 1.0),
                feedforward.calculateFeedforward(new Translation2d(0.5, 0.5), new Translation2d(0.5, 0.5)));
    }
}
