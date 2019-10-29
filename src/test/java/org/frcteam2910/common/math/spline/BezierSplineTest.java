package org.frcteam2910.common.math.spline;

import org.frcteam2910.common.math.Vector2;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class BezierSplineTest {
    @Test
    public void verifyStartEnd() {
        final Vector2 start = new Vector2(120.0, 160.0);
        final Vector2 end = new Vector2(220.0, 40.0);

        final Vector2[] controlPoints = {
                new Vector2(35.0, 200.0),
                new Vector2(220.0, 260.0)
        };

        BezierSpline spline = new BezierSpline(start, end, controlPoints);

        assertEquals(start, spline.getPoint(0.0));
        assertEquals(end, spline.getPoint(1.0));
    }
}
