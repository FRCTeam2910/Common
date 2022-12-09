package org.frcteam2910.common.math.spline;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class CubicBezierSplineTest {
    @Test
    public void verifyStartEndPosition() {
        CubicBezierSpline spline = new CubicBezierSpline(
                new Translation2d(0, 0),
                new Translation2d(5, 0),
                new Translation2d(45, 50),
                new Translation2d(50, 50)
        );
        assertEquals("Starting position is not correct", new Translation2d(0, 0), spline.getPoint(0.0));
        assertEquals("Ending position is not correct", new Translation2d(50, 50), spline.getPoint(1.0));

        spline = new CubicBezierSpline(
                new Translation2d(-20, 35),
                new Translation2d(10, 35),
                new Translation2d(40, -20),
                new Translation2d(50, -20)
        );
        assertEquals("Starting position is not correct", new Translation2d(-20, 35), spline.getPoint(0.0));
        assertEquals("Ending position is not correct", new Translation2d(50, -20), spline.getPoint(1.0));
    }

    @Test
    public void verifyStartEndHeading() {
        CubicBezierSpline spline = new CubicBezierSpline(
                new Translation2d(0, 0),
                new Translation2d(5, 0),
                new Translation2d(45, 50),
                new Translation2d(50, 50)
        );
        assertEquals("Starting heading is not correct", new Rotation2d(), spline.getHeading(0.0));
        assertEquals("Ending heading is not correct", new Rotation2d(), spline.getHeading(1.0));

        spline = new CubicBezierSpline(
                new Translation2d(0, 0),
                new Translation2d(0, 5),
                new Translation2d(40, 40),
                new Translation2d(50, 50)
        );
        assertEquals("Starting heading is not correct", Rotation2d.fromDegrees(90), spline.getHeading(0.0));
        assertEquals("Ending heading is not correct", Rotation2d.fromDegrees(45), spline.getHeading(1.0));
    }
}
