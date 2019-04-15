package org.frcteam2910.common.math;

import java.text.DecimalFormat;

/**
 * This represents a rotation in 3d space through a rotation matrix and its corresponding euler angles
 */
public class Rotation3 {

    /**
     * The rotation matrix of the rotation
     */
    public final double[][] rotationMatrix;

    /**
     * The euler angles from the rotation matrix
     */
    public final double[] eulerAngles;

    /**
     * Create a new Rotation3 from a rotation matrix
     * @param rotationMatrix The rotation matrix to create the rotation form
     */
    public Rotation3(double[][] rotationMatrix) {
        this.rotationMatrix = rotationMatrix;
        eulerAngles = getEulerAngles(rotationMatrix);
    }

    /**
     * Create a new Rotation3 from a set of euler angles
     * @param eulerAngles A double array of euler angles
     */
    public Rotation3(double[] eulerAngles) {
        this.eulerAngles = eulerAngles;
        rotationMatrix = getRotationMatrix(eulerAngles);
    }

    /**
     * Adds this rotation and another rotation together
     * @param other The rotation to add
     * @return A rotation with the result of the addition
     */
    public Rotation3 add(Rotation3 other) {
        return new Rotation3(multiplyMatrices(this.rotationMatrix, other.rotationMatrix));
    }

    /**
     * Multiplies two matrices together
     * @param a The first matrix
     * @param b The second one
     * @return A new matrix which is the product of the two given matrices
     */
    public static double[][] multiplyMatrices(double[][] a, double[][] b) {
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

    /**
     * Calculates the transpose of the rotation matrix, which is equal to it's inverse
     * @return The transpose (inverse) of the rotation matrix
     */
    public Rotation3 inverse() {
        return new Rotation3(transpose(rotationMatrix));
    }

    /**
     * Private method which gets the transpose of a rotation matrix
     * @param a The rotation matrix
     * @return The transpose of the rotation matrix
     */
    private static double[][] transpose(double a[][]) {
        int i, j;
        double[][] b = new double[a.length][a.length];
        for (i = 0; i < a.length; i++) {
            for (j = 0; j < a.length; j++) {
                b[i][j] = a[j][i];
            }
        }
        return b;
    }


    /**
     * Returns a rotation matrix made from the provided euler angles
     * @param eulerAngles The euler angles to get create the rotation matrix from
     * @return The resulting rotation matrix
     */
    private static double[][] getRotationMatrix(double[] eulerAngles) {
        double[][] result_x = new double[][] {
                {1, 0, 0},
                {0, Math.cos(eulerAngles[0]), -Math.sin(eulerAngles[0])},
                {0, Math.sin(eulerAngles[0]), Math.cos(eulerAngles[0])}
        };

        double[][] result_y = new double[][] {
                {Math.cos(eulerAngles[1]), 0, Math.sin(eulerAngles[1])},
                {0, 1, 0},
                {-Math.sin(eulerAngles[1]), 0, Math.cos(eulerAngles[1])}
        };

        double [][] result_z = new double[][] {
                {Math.cos(eulerAngles[2]), -Math.sin(eulerAngles[2]), 0},
                {Math.sin(eulerAngles[2]), Math.cos(eulerAngles[2]), 0},
                {0, 0, 1}
        };
        
        return multiplyMatrices(result_z, multiplyMatrices(result_y, result_x));
    }

    /**
     * Private method which gets euler angles from a rotation matrix
     * @param rotationMatrix The rotation matrix to obtain the euler angles from
     * @return An array of the euler angles
     */
    private static double[] getEulerAngles(double[][] rotationMatrix) {
        double sy = Math.sqrt(Math.pow(rotationMatrix[0][0], 2) + Math.pow(rotationMatrix[1][0], 2));
        boolean singular = sy < 1e-6;
        double x, y, z;
        if (singular) {
            x = Math.atan2(-rotationMatrix[1][2], rotationMatrix[1][1]);
            y = Math.atan2(-rotationMatrix[2][0], sy);
            z = 0;
        } else {
            x = Math.atan2(rotationMatrix[2][1], rotationMatrix[2][2]);
            y = Math.atan2(-rotationMatrix[2][0], sy);
            z = Math.atan2(rotationMatrix[1][0], rotationMatrix[0][0]);
        }
        return new double[] {x, y, z};
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        DecimalFormat fmt = new DecimalFormat("#0.000");
        return "Rotation " +
                "- X: " + fmt.format(Math.toDegrees(eulerAngles[0])) + '\u00b0' +
                ", Y: " + fmt.format(Math.toDegrees(eulerAngles[1])) + '\u00b0' +
                ", Z: " + fmt.format(Math.toDegrees(eulerAngles[2])) + '\u00b0';
    }
}
