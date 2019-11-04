package org.frcteam2910.common.math.spline;

import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class BezierSplineHelperTest {
    @Test
    public void verifyOrder2BezierMatrix() {
        SimpleMatrix expected = new SimpleMatrix(new double[][]{
                new double[]{1, 0, 0},
                new double[]{-2, 2, 0},
                new double[]{1, -2, 1}
        });

        SimpleMatrix actual = BezierSplineHelper.createBasisMatrix(2);

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

        SimpleMatrix actual = BezierSplineHelper.createBasisMatrix(3);

        assertEquals(expected.toString(), actual.toString());
    }

    @Test
    public void verifyOrder4BezierMatrix() {
        SimpleMatrix expected = new SimpleMatrix(new double[][]{
                new double[]{1, 0, 0, 0, 0},
                new double[]{-4, 4, 0, 0, 0},
                new double[]{6, -12, 6, 0, 0},
                new double[]{-4, 12, -12, 4, 0},
                new double[]{1, -4, 6, -4, 1}
        });

        SimpleMatrix actual = BezierSplineHelper.createBasisMatrix(4);

        assertEquals(expected.toString(), actual.toString());
    }
}
