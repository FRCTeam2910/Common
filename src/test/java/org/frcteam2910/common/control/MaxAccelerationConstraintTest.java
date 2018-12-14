package org.frcteam2910.common.control;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Vector2;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

public class MaxAccelerationConstraintTest {
    private static final PathSegment[] SEGMENTS = {
            new PathLineSegment(Vector2.ZERO, new Vector2(1.0, 0.0)),
            new PathLineSegment(new Vector2(1.0, 0.0), Vector2.ZERO),
            new PathArcSegment(Vector2.ZERO, new Vector2(5.0, 5.0), new Vector2(5.0, 0.0))
    };

    private static final double[] VELOCITIES = {
            5.0, 1.0, 3.0, 25.0, Double.POSITIVE_INFINITY
    };

    private static final double[] MAX_ACCELERATIONS = {
            5.0, 10.0, 3.5, Math.PI
    };

    @Test
    public void constructorTest() {
        for (double maxAcceleration : MAX_ACCELERATIONS) {
            try {
                MaxAccelerationConstraint constraint = new MaxAccelerationConstraint(maxAcceleration);
            } catch (IllegalArgumentException e) {
                fail("Threw exception when max acceleration was positive");
            }

            try {
                MaxAccelerationConstraint constraint = new MaxAccelerationConstraint(-maxAcceleration);
                fail("Did not throw exception when max acceleration was negative");
            } catch (IllegalArgumentException e) {
                // Do nothing, this is expected.
            }
        }
    }

    @Test
    public void getMaxAccelerationTest() {
        for (PathSegment segment : SEGMENTS) {
            for (double velocity : VELOCITIES) {
                for (double maxAcceleration : MAX_ACCELERATIONS) {
                    MaxAccelerationConstraint constraint = new MaxAccelerationConstraint(maxAcceleration);
                    assertEquals("Not constraining acceleration correctly", maxAcceleration,
                            constraint.getMaxAcceleration(segment, velocity), MathUtils.EPSILON);
                }
            }
        }
    }

    @Test
    public void getMaxVelocityTest() {
        for (PathSegment segment : SEGMENTS) {
            for (double maxAcceleration : MAX_ACCELERATIONS) {
                MaxAccelerationConstraint constraint = new MaxAccelerationConstraint(maxAcceleration);
                assertEquals("Constraining velocity when it should not be constrained", Double.POSITIVE_INFINITY,
                        constraint.getMaxVelocity(segment), MathUtils.EPSILON);
            }
        }
    }
}
