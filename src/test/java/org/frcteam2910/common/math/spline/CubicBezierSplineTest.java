package org.frcteam2910.common.math.spline;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class CubicBezierSplineTest {
    @Test
    void verifyStartEndPosition() {
        CubicBezierSpline spline = new CubicBezierSpline(
                new Translation2d(0, 0), new Translation2d(5, 0), new Translation2d(45, 50), new Translation2d(50, 50));
        assertEquals(new Translation2d(0, 0), spline.getPoint(0.0), "Starting position is not correct");
        assertEquals(new Translation2d(50, 50), spline.getPoint(1.0), "Ending position is not correct");

        spline = new CubicBezierSpline(
                new Translation2d(-20, 35),
                new Translation2d(10, 35),
                new Translation2d(40, -20),
                new Translation2d(50, -20));
        assertEquals(new Translation2d(-20, 35), spline.getPoint(0.0), "Starting position is not correct");
        assertEquals(new Translation2d(50, -20), spline.getPoint(1.0), "Ending position is not correct");
    }

    @Test
    void verifyStartEndHeading() {
        CubicBezierSpline spline = new CubicBezierSpline(
                new Translation2d(0, 0), new Translation2d(5, 0), new Translation2d(45, 50), new Translation2d(50, 50));
        assertEquals(new Rotation2d(), spline.getHeading(0.0), "Starting heading is not correct");
        assertEquals(new Rotation2d(), spline.getHeading(1.0), "Ending heading is not correct");

        spline = new CubicBezierSpline(
                new Translation2d(0, 0), new Translation2d(0, 5), new Translation2d(40, 40), new Translation2d(50, 50));
        assertEquals(Rotation2d.fromDegrees(90), spline.getHeading(0.0), "Starting heading is not correct");
        assertEquals(Rotation2d.fromDegrees(45), spline.getHeading(1.0), "Ending heading is not correct");
    }
}
