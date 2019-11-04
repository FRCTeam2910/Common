package org.frcteam2910.common.math.spline;

import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class SplineTest {
    @Test
    public void derivative() {
        // Use the polynomials:
        // x(t) = 5x^3 - 20x^2 + x - 9
        // y(t) = -52x^3 - 7x + 2
        //
        // NOTE: Coefficients have to be put in the matrix in reverse order. I.e. x^3 should be in the last row, x^2 in
        // the second to last, and so on.
        //
        // Because we are just putting our coefficients into the weight matrix, the basis matrix will the the identity
        // matrix.
        Spline spline = new Spline(
                SimpleMatrix.identity(4),
                new SimpleMatrix(new double[][]{
                        new double[]{-9, 2},
                        new double[]{1, -7},
                        new double[]{-20, 0},
                        new double[]{5, -52}
                })
        );

        // Based off of the given polynomials, we would expect the derivatives to be the following:
        // x'(t) = 15x^2 - 40x + 1
        // y'(t) = -156x^2 - 7
        SimpleMatrix dcoefficients = new SimpleMatrix(new double[][]{
                new double[]{1, -7},
                new double[]{-40, 0},
                new double[]{15, -156}
        });
        Spline dspline = spline.derivative();

        assertEquals(dcoefficients.toString(), dspline.getBasisWeightMatrix().toString());
    }
}
