package org.frcteam2910.common.math.spline;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class CubicHermiteSplineTest {
    @Test
    void verifyStartEndPosition() {
        CubicHermiteSpline spline = new CubicHermiteSpline(
                new Translation2d(0, 0), new Rotation2d(),
                new Translation2d(50, 50), new Rotation2d());
        assertEquals(new Translation2d(0, 0), spline.getPoint(0.0), "Starting position is not correct");
        assertEquals(new Translation2d(50, 50), spline.getPoint(1.0), "Ending position is not correct");

        spline = new CubicHermiteSpline(
                new Translation2d(-20, 35), Rotation2d.fromDegrees(90),
                new Translation2d(50, -20), Rotation2d.fromDegrees(45));
        assertEquals(new Translation2d(-20, 35), spline.getPoint(0.0), "Starting position is not correct");
        assertEquals(new Translation2d(50, -20), spline.getPoint(1.0), "Ending position is not correct");
    }

    @Test
    void verifyStartEndHeading() {
        Spline spline = new CubicHermiteSpline(
                new Translation2d(0, 0), new Rotation2d(),
                new Translation2d(50, 50), new Rotation2d());
        assertEquals(new Rotation2d(), spline.getHeading(0.0), "Starting heading is not correct");
        assertEquals(new Rotation2d(), spline.getHeading(1.0), "Ending heading is not correct");

        spline = new CubicHermiteSpline(
                new Translation2d(0, 0), Rotation2d.fromDegrees(90),
                new Translation2d(50, 50), Rotation2d.fromDegrees(45));
        assertEquals(Rotation2d.fromDegrees(90), spline.getHeading(0.0), "Starting heading is not correct");
        assertEquals(Rotation2d.fromDegrees(45), spline.getHeading(1.0), "Ending heading is not correct");
    }
}
