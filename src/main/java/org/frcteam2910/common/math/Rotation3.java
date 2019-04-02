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
        this.eulerAngles = getEulerAngles(rotationMatrix);
    }

    /**
     * Adds this rotation and another rotation together
     * @param other The rotation to add
     * @return A rotation with the result of the addition
     */
    public Rotation3 add(Rotation3 other) {
        return add(other.rotationMatrix);
    }

    /**
     * Adds another rotation matrix to this rotation
     * @param other The rotation matrix to add
     * @return A rotation with the result of the addition
     */
    public Rotation3 add(double[][] other) {
        double[][] result = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result[i][j] = this.rotationMatrix[i][j] + other[i][j];
            }
        }
        return new Rotation3(result);
    }

    /**
     * Calculates the transpose of the rotation matrix, which is equal to it's inverse
     * @return The transpose (inverse) of the rotation matrix
     */
    public Rotation3 inverse() {
        return new Rotation3(transpose(rotationMatrix));
    }

    /**
     * Private method which gets euler angles from a rotation matrix
     * @param rotationMatrix The rotation matrix to obtain the euler angles from
     * @return An array of the euler angles
     */
    private static double[] getEulerAngles(double[][] rotationMatrix) {
        double sy = Math.sqrt(Math.pow(rotationMatrix[0][0], 2) + Math.pow(rotationMatrix[1][0], 2));
        boolean singular = sy < 1e-6;
        double x;
        double y;
        double z;
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
     * Private method which gets the transpose of a rotation matrix
     * @param a The rotation matrix
     * @return The transpose of the rotation matrix
     */
    private static double[][] transpose(double a[][])
    {
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
