package org.frcteam2910.common.math.spline;

import org.junit.Test;

import static org.frcteam2910.common.math.spline.SplineHelper.binomial;
import static org.junit.Assert.assertEquals;

public class SplineHelperTest {
    @Test
    public void verifyBinomial() {
        int[][] binomialLookupTable = {
                {1},
                {1, 1},
                {1, 2, 1},
                {1, 3, 3, 1},
                {1, 4, 6, 4, 1},
                {1, 5, 10, 10, 5, 1},
                {1, 6, 15, 20, 15, 6, 1}
        };

        for (int n = 0; n < binomialLookupTable.length; n++) {
            for (int k = 0; k <= n; k++) {
                assertEquals(String.format("Binomial (%d %d) is not correct", n, k), binomialLookupTable[n][k], binomial(n, k));
            }
        }
    }
}
