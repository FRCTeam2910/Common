package org.frcteam2910.common.math;

public class MatrixUtil {
    /**
     * Multiplies two matrices together
     * @param a The first matrix
     * @param b The second one
     * @return A new matrix which is the product of the two given matrices
     */
    public static double[][] multiplyMatrices(double[][] a, double[][] b) throws IllegalArgumentException {
        if (a[0].length != b.length) {
            throw new IllegalArgumentException("The number of columns of the first matrix must be equal to the number of rows in the second matrix");
        }
        int r1 = a.length;
        int c1 = a[0].length;
        int c2 = b[0].length;
        double[][] product = new double[r1][c2];
        for (int i = 0; i < r1; i++) {
            for (int j = 0; j < c2; j++) {
                for (int k = 0; k < c1; k++) {
                    product[i][j] += a[i][k] * b[k][j];
                }
            }
        }
        return product;
    }
}
