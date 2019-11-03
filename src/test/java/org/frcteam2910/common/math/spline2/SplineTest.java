package org.frcteam2910.common.math.spline2;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.math.spline.BezierSpline;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class SplineTest {
    @Test
    public void verifyStartEndHeading() {
        Spline spline = new CubicBezierSpline(
                new Vector2(0, 0),
                new Vector2(5, 0),
                new Vector2(45, 50),
                new Vector2(50, 50)
        );
        assertEquals("Starting heading is not correct", Rotation2.ZERO, spline.getHeading(0.0));
        assertEquals("Ending heading is not correct", Rotation2.ZERO, spline.getHeading(1.0));

        spline = new CubicBezierSpline(
                new Vector2(0, 0),
                new Vector2(0, 5),
                new Vector2(40, 40),
                new Vector2(50, 50)
        );
        assertEquals("Starting heading is not correct", Rotation2.fromDegrees(90), spline.getHeading(0.0));
        assertEquals("Ending heading is not correct", Rotation2.fromDegrees(45), spline.getHeading(1.0));
    }

    @Test
    public void derivative() {
        Spline spline = new CubicBezierSpline(
                new Vector2(0, 0),
                new Vector2(5, 0),
                new Vector2(45, 50),
                new Vector2(50, 50)
        );

        Spline dspline = spline.derivative();

        BezierSpline ds = new BezierSpline(
                new Vector2[]{
                        new Vector2(0, 0),
                        new Vector2(5, 0),
                        new Vector2(45, 50),
                        new Vector2(50, 50)
                }
        ).derivative();

        for (int i = 0; i <= 100; i++) {
            double t = i * 0.01;
            assertEquals(String.format("Derivative incorrect at t = %.2f", t), ds.getPoint(t), dspline.getPoint(t));
        }
    }
}
