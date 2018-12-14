package org.frcteam2910.common.control;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Vector2;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

public class FeedforwardConstraintTest {
    /**
     * Format is {target, kV, kA, kS}
     */
    private static final double[][] CONSTANTS = {
            {9.0, 0.8, 0.1, 0.6},
            {12.0, 1.325, 0.129, 0.798},
            {10.5, 0.660, 0.122, 0.766},
            {10.5, 0.648, 0.141, 0.774},
    };

    private static final PathSegment[] SEGMENTS = {
            new PathLineSegment(Vector2.ZERO, new Vector2(1.0, 0.0)),
            new PathLineSegment(new Vector2(1.0, 0.0), Vector2.ZERO),
            new PathArcSegment(Vector2.ZERO, new Vector2(5.0, 5.0), new Vector2(5.0, 0.0))
    };

    @Test
    public void constructorTest() {
        for (double[] constants : CONSTANTS) {
            try {
                new FeedforwardConstraint(constants[0], constants[1], constants[2], constants[3]);
            } catch (IllegalArgumentException e) {
                fail("Threw exception when constants were valid");
            }

            try {
                new FeedforwardConstraint(-constants[0], constants[1], constants[2], constants[3]);
                fail("Did not throw exception when target was negative");
            } catch (IllegalArgumentException e) {
                // Do nothing, this is expected.
            }

            try {
                new FeedforwardConstraint(constants[0], -constants[1], constants[2], constants[3]);
                fail("Did not throw exception when kV was negative");
            } catch (IllegalArgumentException e) {
                // Do nothing, this is expected.
            }

            try {
                new FeedforwardConstraint(constants[0], constants[1], -constants[2], constants[3]);
                fail("Did not throw exception when kA was negative");
            } catch (IllegalArgumentException e) {
                // Do nothing, this is expected.
            }

            try {
                new FeedforwardConstraint(constants[0], constants[1], constants[2], -constants[3]);
                fail("Did not throw exception when kS was negative");
            } catch (IllegalArgumentException e) {
                // Do nothing, this is expected.
            }
        }
    }

    @Test
    public void targetNotExceededTest() {
        for (double[] constants : CONSTANTS) {
            ITrajectoryConstraint constraint = new FeedforwardConstraint(constants[0], constants[1], constants[2],
                    constants[3]);
            for (PathSegment segment : SEGMENTS) {
                double velocity = constraint.getMaxVelocity(segment);
                double acceleration = constraint.getMaxAcceleration(segment, velocity);

                assertEquals("Target feedforward is not met", constants[0],
                        constants[1] * velocity + constants[2] * acceleration + constants[3], MathUtils.EPSILON);
            }
        }
    }
}
