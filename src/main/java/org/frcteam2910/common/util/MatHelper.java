package org.frcteam2910.c2019.vision.util;

import org.opencv.core.*;

/**
 * A class with useful function when working with OpenCV matrices
 */
public class MatHelper {

    /**
     * Converts a matrix to a point. The matrix, of course, must represent a 2D point
     * @param mat The matrix to convert
     * @return The Point representation of the Matrix
     */
    public static Point matToPoint(Mat mat) {
        return new Point(mat.get(0, 0)[0], mat.get(0, 1)[0]);
    }

    /**
     * Converts a matrix to a point. The matrix, of course, must represent a 3D point
     * @param mat The matrix to convert
     * @return The Point3 representation of the matrix
     */
    public static Point3 matToPoint3(Mat mat) {
        return new Point3(mat.get(0, 0)[0], mat.get(0, 1)[0], mat.get(0, 2)[0]);
    }

    /**
     * Converts a 2 dimensional double array to a matrix
     * @param data The 2 dimensional double array to convert
     * @return The Matrix representation of the 2 dimensional array
     */
    public static Mat multiDimensionalDoubleArrayToMat(double[][] data) {
        Mat result = new Mat(data.length, data[0].length, CvType.CV_32F);
        for (int x = 0; x < data.length; x++) {
            result.put(x, 0, data[x]);
        }
        return result;
    }

    /**
     * Converts a double array to a matrix
     * @param data The double array to convert
     * @return The Matrix representation of the double array
     */
    public static Mat doubleArrayToMat(double[] data) {
        Mat result = new Mat(1, data.length, CvType.CV_32F);
        for (int i = 0; i < data.length; i++) {
            result.put(0, i, data[i]);
        }
        return result;
    }

    /**
     * Converts a matrix to a 2 dimensional double array
     * @param data The matrix to convert
     * @return The 2 dimensional double array of the Matrix
     */
    public static double[][] matTo2DDoubleArray(Mat data) {
        Size size = data.size();
        double[][] result = new double[(int) size.height][(int) size.width];
        for (int i = 0; i < size.height; i++) {
            for (int j = 0; j < size.width; j++) {
                result[i][j] = data.get(i, j)[0];
            }
        }
        return result;
    }

    /**
     * Converts a 2D point to a double array
     * @param pt The point to convert
     * @return The double array representation of the point
     */
    public static double[] pointToDoubleArray(Point pt) {
        double[] result = new double[2];
        result[0] = pt.x;
        result[1] = pt.y;
        return result;
    }

    /**
     * Converts a matrix to a double array
     * @param mat The matrix to convert
     * @return The resulting double array
     */
    public static double[] matToDoubleArray(Mat mat) {
        double[] result = new double[(int) mat.size().width];
        for (int i = 0; i < mat.size().width; i++) {
            result[i] = mat.get(0, i)[0];
        }
        return result;
    }

    /**
     * Converts a matrix to a Matrix of 2D Points
     * @param mat The matrix to convert
     * @return The Matrix of 2D points
     */
    public static MatOfPoint2f matToMatOfPoint2f(Mat mat) {
        Point[] points = new Point[(int) mat.size().height];
        for (int i = 0; i < mat.size().height; i++) {
            points[i] = new Point(mat.get(i, 0)[0], mat.get(i, 1)[0]);
        }
        return new MatOfPoint2f(points);
    }

    /**
     * Converts a 2 dimensional double array to a MatOfPoint2f
     * @param corners The 2 dimensional double array to be converted
     * @return returns a MatOfPoint2f
     */
    public static MatOfPoint2f doubleArrayToMatOfPoint2f(double[][] corners) {
        Point[] points = new Point[corners.length];
        for (int i = 0; i < corners.length; i++) {
            points[i] = new Point(corners[i][0], corners[i][1]);
        }
        return new MatOfPoint2f(points);
    }

    /**
     * Converts a 2 dimensional double array into a MatOfPoint3f
     * @param corners The 2 dimensional double array to be converted
     * @return returns a MatOfPoint3f
     */
    public static MatOfPoint3f doubleArrayToMatOfPoint3f(double[][] corners) {
        Point3[] points = new Point3[corners.length];
        for (int i = 0; i < corners.length; i++) {
            points[i] = new Point3(corners[i][0], corners[i][1], 0);
        }
        return new MatOfPoint3f(points);
    }

    public static Mat multiDimensionalDoubleArrayToMat(double[][] data, int type) {
        Size size = new Size(data.length, data[0].length);
        Mat mat = new Mat(size, type);
        for (int i = 0; i < size.height; i++) {
            for (int j = 0; j < size.width; j++) {
                mat.put(i, j, new double[] {data[i][j]});
            }
        }
        return mat;
    }
}
