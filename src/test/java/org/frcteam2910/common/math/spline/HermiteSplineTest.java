package org.frcteam2910.common.math.spline;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class HermiteSplineTest {
    @Test
    public void verifyStartEndCubicTest() {
        HermiteSpline spline = HermiteSpline.cubic(new RigidTransform2(Vector2.ZERO, Rotation2.ZERO),
                new RigidTransform2(new Vector2(12.0, 12.0), Rotation2.ZERO));

        assertEquals("Starting position does not match desired position",
                Vector2.ZERO, spline.getPoint(0.0));
        assertEquals("Starting heading does not match desired heading",
                Rotation2.ZERO, spline.getHeading(0.0));
        assertEquals("Ending position does not match desired position",
                new Vector2(12.0, 12.0), spline.getPoint(1.0));
        assertEquals("Ending heading does not match desired heading",
                Rotation2.ZERO, spline.getHeading(1.0));
    }

    @Test
    public void verifyStartEndQuinticTest() {
        HermiteSpline spline = HermiteSpline.quintic(new RigidTransform2(Vector2.ZERO, Rotation2.ZERO),
                new RigidTransform2(new Vector2(12.0, 12.0), Rotation2.ZERO));

        assertEquals("Starting position does not match desired position",
                Vector2.ZERO, spline.getPoint(0.0));
        assertEquals("Starting heading does not match desired heading",
                Rotation2.ZERO, spline.getHeading(0.0));
        assertEquals("Ending position does not match desired position",
                new Vector2(12.0, 12.0), spline.getPoint(1.0));
        assertEquals("Ending heading does not match desired heading",
                Rotation2.ZERO, spline.getHeading(1.0));
    }
}
