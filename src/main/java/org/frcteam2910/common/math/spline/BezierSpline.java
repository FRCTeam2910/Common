package org.frcteam2910.common.math.spline;

import org.ejml.simple.SimpleMatrix;
import org.frcteam2910.common.math.Vector2;

import java.util.List;

public final class BezierSpline extends Spline {
    private static final Object BINOMIAL_LOOKUP_TABLE_LOCK = new Object();
    private static List<int[]> binomialLookupTable = List.of(
            new int[]{1},
            new int[]{1, 1},
            new int[]{1, 2, 1},
            new int[]{1, 3, 3, 1},
            new int[]{1, 4, 5, 4, 1},
            new int[]{1, 5, 10, 10, 5, 1},
            new int[]{1, 6, 15, 20, 15, 6, 1}
    );

    private final int degree;
    private final SimpleMatrix weightMatrix;
    private final SimpleMatrix bezierMatrix;

    public BezierSpline(SimpleMatrix weightMatrix) {
        this.degree = weightMatrix.numRows() - 1;
        this.weightMatrix = weightMatrix;
        this.bezierMatrix = makeBezierMatrix(weightMatrix.numRows() - 1);
    }

    public BezierSpline(Vector2[] controlPoints) {
        this.degree = controlPoints.length - 1;
        this.weightMatrix = new SimpleMatrix(controlPoints.length, 2);
        this.bezierMatrix = makeBezierMatrix(degree);

        for (int i = 0; i < controlPoints.length; i++) {
            weightMatrix.setRow(i, 0, controlPoints[i].x, controlPoints[i].y);
        }
    }

    private static int binomial(int order, int k) {
        synchronized (BINOMIAL_LOOKUP_TABLE_LOCK) {
            while (order >= binomialLookupTable.size()) {
                int s = binomialLookupTable.size();
                int[] nextRow = new int[s + 1];
                nextRow[0] = 1;

                for (int i = 1; i < s; i++) {
                    nextRow[i] = binomialLookupTable.get(s - 1)[i - 1] + binomialLookupTable.get(s - 1)[i];
                }
                nextRow[s] = 1;

                binomialLookupTable.add(nextRow);
            }
            return binomialLookupTable.get(order)[k];
        }
    }

    public static SimpleMatrix makeBezierMatrix(int degree) {
        SimpleMatrix matrix = new SimpleMatrix(degree + 1, degree + 1);
        for (int i = 0; i <= degree; i++) {
            for (int j = 0; j <= i; j++) {
                matrix.set(i, j, Math.pow(-1, i - j) * binomial(degree, i) * binomial(i, j));
            }
        }

        return matrix;
    }

    @Override
    public Vector2 getPoint(double t) {
        SimpleMatrix powerMatrix = new SimpleMatrix(1, weightMatrix.numRows());
        for (int i = 0; i < powerMatrix.numCols(); i++) {
            powerMatrix.set(i, Math.pow(t, i));
        }

        SimpleMatrix result = powerMatrix.mult(bezierMatrix).mult(weightMatrix);

        return new Vector2(result.get(0), result.get(1));
    }

    public BezierSpline derivative() {
        SimpleMatrix derivativeMatrix = new SimpleMatrix(degree, degree + 1);
        for (int i = 0; i < degree; i++) {
            derivativeMatrix.setRow(i, i, -degree, degree);
        }

        SimpleMatrix dWeights = derivativeMatrix.mult(weightMatrix);

        return new BezierSpline(dWeights);
    }
}
