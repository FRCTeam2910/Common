package org.frcteam2910.common.math.spline;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class CubicHermiteSplineTest {
    @Test
    public void verifyStartEndPosition() {
        CubicHermiteSpline spline = new CubicHermiteSpline(
                new Vector2(0, 0), Rotation2.ZERO,
                new Vector2(50, 50), Rotation2.ZERO
        );
        assertEquals("Starting position is not correct", new Vector2(0, 0), spline.getPoint(0.0));
        assertEquals("Ending position is not correct", new Vector2(50, 50), spline.getPoint(1.0));

        spline = new CubicHermiteSpline(
                new Vector2(-20, 35), Rotation2.fromDegrees(90),
                new Vector2(50, -20), Rotation2.fromDegrees(45)
        );
        assertEquals("Starting position is not correct", new Vector2(-20, 35), spline.getPoint(0.0));
        assertEquals("Ending position is not correct", new Vector2(50, -20), spline.getPoint(1.0));
    }

    @Test
    public void verifyStartEndHeading() {
        Spline spline = new CubicHermiteSpline(
                new Vector2(0, 0), Rotation2.ZERO,
                new Vector2(50, 50), Rotation2.ZERO
        );
        assertEquals("Starting heading is not correct", Rotation2.ZERO, spline.getHeading(0.0));
        assertEquals("Ending heading is not correct", Rotation2.ZERO, spline.getHeading(1.0));

        spline = new CubicHermiteSpline(
                new Vector2(0, 0), Rotation2.fromDegrees(90),
                new Vector2(50, 50), Rotation2.fromDegrees(45)
        );
        assertEquals("Starting heading is not correct", Rotation2.fromDegrees(90), spline.getHeading(0.0));
        assertEquals("Ending heading is not correct", Rotation2.fromDegrees(45), spline.getHeading(1.0));
    }
}
