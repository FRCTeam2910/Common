package org.frcteam2910.common.math.spline;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.List;

class SplineHelper {
    private static final Object BINOMIAL_LOOKUP_TABLE_LOCK = new Object();
    private static List<int[]> binomialLookupTable = new ArrayList<>();

    private SplineHelper() {
    }

    public static int binomial(int order, int k) {
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

    public static SimpleMatrix createPowerMatrix(int degree, double t) {
        if (degree < 0) {
            throw new IllegalArgumentException("Degree cannot be less than 0");
        }

        SimpleMatrix powerMatrix = new SimpleMatrix(1, degree + 1);
        powerMatrix.set(0, 1);
        for (int i = 1; i <= degree; i++) {
            powerMatrix.set(i, powerMatrix.get(i - 1) * t);
        }

        return powerMatrix;
    }
}
