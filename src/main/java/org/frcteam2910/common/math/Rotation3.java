package org.frcteam2910.common.math;

import org.opencv.core.Mat;

import java.text.DecimalFormat;

import static org.frcteam2910.common.util.MatHelper.*;
import static org.opencv.core.Core.gemm;
import static org.opencv.core.Core.transpose;

/**
 * This represents a rotation in 3d space through a rotation matrix and its corresponding euler angles
 */
public class Rotation3 {

    /**
     * The rotation matrix of the rotation
     */
    public final Mat rotationMatrix;

    /**
     * The euler angles from the rotation matrix
     */
    public final Mat eulerAngles;

    /**
     * Create a new Rotation3 from a rotation matrix
     * @param rotationMatrix The rotation matrix to create the rotation form
     */
    public Rotation3(double[][] rotationMatrix) {
        this.rotationMatrix = multiDimensionalDoubleArrayToMat(rotationMatrix);
        this.eulerAngles = doubleArrayToMat(getEulerAngles(rotationMatrix));
    }

    /**
     * Create a new Rotation3 from a set of euler angles
     * @param eulerAngles A double array of euler angles
     */
    public Rotation3(double[] eulerAngles) {
        this.eulerAngles = doubleArrayToMat(eulerAngles);
        rotationMatrix = multiDimensionalDoubleArrayToMat(getRotationMatrix(eulerAngles));
    }

    /**
     * Create a new Rotation3 from a rotation matrix
     * @param rotationMatrix The rotation matrix as a OpenCV matrix
     */
    public Rotation3(Mat rotationMatrix) {
        this.rotationMatrix = rotationMatrix;
        eulerAngles = doubleArrayToMat(getEulerAngles(matTo2DDoubleArray(rotationMatrix)));
    }

    /**
     * Adds this rotation and another rotation together
     * @param other The rotation to add
     * @return A rotation with the result of the addition
     */
    public Rotation3 add(Rotation3 other) {
        return new Rotation3(rotationMatrix.mul(other.rotationMatrix));
    }

    /**
     * Multiplies a rotation 
     * @param vector
     * @return
     */
    public Rotation3 multiply(Vector3 vector) {

    }

    /**
     * Calculates the transpose of the rotation matrix, which is equal to it's inverse
     * @return The transpose (inverse) of the rotation matrix
     */
    public Rotation3 inverse() {
        Mat inverse = new Mat();
        transpose(rotationMatrix, inverse);
        return new Rotation3(inverse);
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

        Mat x = multiDimensionalDoubleArrayToMat(result_x);
        Mat y = multiDimensionalDoubleArrayToMat(result_y);
        Mat z = multiDimensionalDoubleArrayToMat(result_z);
        Mat result = new Mat();
        gemm(x, y, 1, z, 0, result);
        return matTo2DDoubleArray(result);
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
        double[] _eulerAngles = matToDoubleArray(eulerAngles);
        DecimalFormat fmt = new DecimalFormat("#0.000");
        return "Rotation " +
                "- X: " + fmt.format(Math.toDegrees(_eulerAngles[0])) + '\u00b0' +
                ", Y: " + fmt.format(Math.toDegrees(_eulerAngles[1])) + '\u00b0' +
                ", Z: " + fmt.format(Math.toDegrees(_eulerAngles[2])) + '\u00b0';
    }
}
