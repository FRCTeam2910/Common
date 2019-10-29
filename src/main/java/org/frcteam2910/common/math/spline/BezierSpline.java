package org.frcteam2910.common.math.spline;

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

    private final double[] xWeights;
    private final double[] yWeights;

    public BezierSpline(Vector2 start, Vector2 end, Vector2... controlPoints) {
        xWeights = new double[controlPoints.length + 2];
        yWeights = new double[controlPoints.length + 2];

        xWeights[0] = start.x;
        yWeights[0] = start.y;

        for (int i = 0; i < controlPoints.length; i++) {
            xWeights[i + 1] = controlPoints[i].x;
            yWeights[i + 1] = controlPoints[i].y;
        }

        xWeights[xWeights.length - 1] = end.x;
        yWeights[yWeights.length - 1] = end.y;
    }

    public BezierSpline(double[] xWeights, double[] yWeights) {
        assert xWeights.length == yWeights.length;

        this.xWeights = xWeights;
        this.yWeights = yWeights;
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

    private static double basis(int order, double t, double[] weights) {
        double sum = 0.0;
        for (int k = 0; k <= order; k++) {
            sum += weights[k] * binomial(order, k) * Math.pow(1 - t, order - k) * Math.pow(t, k);
        }

        return sum;
    }

    @Override
    public Vector2 getPoint(double t) {
        double x = 0.0;
        double y = 0.0;

        for (int k = 0; k < xWeights.length; k++) {
            double unweightedBasis = binomial(xWeights.length - 1, k) * Math.pow(1 - t, xWeights.length - 1 - k) * Math.pow(t, k);

            x += xWeights[k] * unweightedBasis;
            y += yWeights[k] * unweightedBasis;
        }

        return new Vector2(x, y);
    }

    public BezierSpline derivative() {
        double[] dxWeights = new double[xWeights.length - 1];
        double[] dyWeights = new double[yWeights.length - 1];

        for (int i = 0; i < dxWeights.length; i++) {
            dxWeights[i] = dxWeights.length * (xWeights[i + 1] - xWeights[i]);
            dyWeights[i] = dyWeights.length * (yWeights[i + 1] - yWeights[i]);
        }

        return new BezierSpline(dxWeights, dyWeights);
    }
}
