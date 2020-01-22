package org.frcteam2910.common.math.spline;

import org.ejml.simple.SimpleMatrix;
import org.frcteam2910.common.math.Vector2;

import static org.frcteam2910.common.math.spline.SplineHelper.binomial;

class BezierSplineHelper {
    private BezierSplineHelper() {
    }

    public static SimpleMatrix createBasisMatrix(int degree) {
        SimpleMatrix matrix = new SimpleMatrix(degree + 1, degree + 1);
        for (int i = 0; i <= degree; i++) {
            for (int j = 0; j <= i; j++) {
                matrix.set(i, j, Math.pow(-1, i - j) * binomial(degree, i) * binomial(i, j));
            }
        }

        return matrix;
    }

    public static SimpleMatrix createBasisWeightMatrix(Vector2... controlPoints) {
        SimpleMatrix basisWeightMatrix = new SimpleMatrix(controlPoints.length, 2);
        for (int i = 0; i < controlPoints.length; i++) {
            basisWeightMatrix.setRow(i, 0, controlPoints[i].x, controlPoints[i].y);
        }
        return basisWeightMatrix;
    }

    public static Vector2[] basisWeightMatrixToControlPoints(SimpleMatrix basisWeightMatrix) {
        if (basisWeightMatrix.numCols() != 2) {
            throw new IllegalArgumentException("Basis weight matrix must have 2 columns");
        }

        Vector2[] controlPoints = new Vector2[basisWeightMatrix.numRows()];
        for (int i = 0; i < controlPoints.length; i++) {
            controlPoints[i] = new Vector2(basisWeightMatrix.get(i, 0), basisWeightMatrix.get(i, 1));
        }
        return controlPoints;
    }
}
