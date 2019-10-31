package org.frcteam2910.common.math.spline;

import org.ejml.simple.SimpleMatrix;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class BezierSplineTest {
    @Test
    public void verifyOrder2BezierMatrix() {
        SimpleMatrix expected = new SimpleMatrix(new double[][]{
                new double[]{1, 0, 0},
                new double[]{-2, 2, 0},
                new double[]{1, -2, 1}
        });

        SimpleMatrix actual = BezierSpline.makeBezierMatrix(2);

        assertEquals(expected.toString(), actual.toString());
    }

    @Test
    public void verifyOrder3BezierMatrix() {
        SimpleMatrix expected = new SimpleMatrix(new double[][]{
                new double[]{1, 0, 0, 0},
                new double[]{-3, 3, 0, 0},
                new double[]{3, -6, 3, 0},
                new double[]{-1, 3, -3, 1}
        });

        SimpleMatrix actual = BezierSpline.makeBezierMatrix(3);

        assertEquals(expected.toString(), actual.toString());
    }

    @Test
    public void verifyOrder4BezierMatrix() {
        SimpleMatrix expected = new SimpleMatrix(new double[][]{
                new double[]{1, 0, 0, 0, 0},
                new double[]{-4, 4, 0, 0, 0},
                new double[]{5, -10, 5, 0, 0},
                new double[]{-4, 12, -12, 4, 0},
                new double[]{1, -4, 5, -4, 1}
        });

        SimpleMatrix actual = BezierSpline.makeBezierMatrix(4);

        assertEquals(expected.toString(), actual.toString());
    }

    @Test
    public void verifyStartEnd() {
        final Vector2[] controlPoints = {
                new Vector2(120.0, 160.0),
                new Vector2(35.0, 200.0),
                new Vector2(220.0, 260.0),
                new Vector2(220.0, 40.0)
        };

        final Rotation2 startHeading = controlPoints[1].subtract(controlPoints[0]).getAngle();
        final Rotation2 endHeading = controlPoints[3].subtract(controlPoints[2]).getAngle();

        BezierSpline spline = new BezierSpline(controlPoints);

        assertEquals("Starting position was not correct", controlPoints[0], spline.getPoint(0.0));
        assertEquals("Starting heading was not correct", startHeading, spline.getHeading(0.0));
        assertEquals("Ending position was not correct", controlPoints[3], spline.getPoint(1.0));
        assertEquals("Ending heading was not correct", endHeading, spline.getHeading(1.0));
    }
}
