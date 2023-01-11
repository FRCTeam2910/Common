package org.frcteam2910.common.math.spline;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class CubicHermiteSplineTest {
    @Test
    public void verifyStartEndPosition() {
        CubicHermiteSpline spline = new CubicHermiteSpline(
                new Translation2d(0, 0), new Rotation2d(),
                new Translation2d(50, 50), new Rotation2d()
        );
        assertEquals("Starting position is not correct", new Translation2d(0, 0), spline.getPoint(0.0));
        assertEquals("Ending position is not correct", new Translation2d(50, 50), spline.getPoint(1.0));

        spline = new CubicHermiteSpline(
                new Translation2d(-20, 35), Rotation2d.fromDegrees(90),
                new Translation2d(50, -20), Rotation2d.fromDegrees(45)
        );
        assertEquals("Starting position is not correct", new Translation2d(-20, 35), spline.getPoint(0.0));
        assertEquals("Ending position is not correct", new Translation2d(50, -20), spline.getPoint(1.0));
    }

    @Test
    public void verifyStartEndHeading() {
        Spline spline = new CubicHermiteSpline(
                new Translation2d(0, 0), new Rotation2d(),
                new Translation2d(50, 50), new Rotation2d()
        );
        assertEquals("Starting heading is not correct", new Rotation2d(), spline.getHeading(0.0));
        assertEquals("Ending heading is not correct", new Rotation2d(), spline.getHeading(1.0));

        spline = new CubicHermiteSpline(
                new Translation2d(0, 0), Rotation2d.fromDegrees(90),
                new Translation2d(50, 50), Rotation2d.fromDegrees(45)
        );
        assertEquals("Starting heading is not correct", Rotation2d.fromDegrees(90), spline.getHeading(0.0));
        assertEquals("Ending heading is not correct", Rotation2d.fromDegrees(45), spline.getHeading(1.0));
    }
}
