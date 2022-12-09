package org.frcteam2910.common.math.spline;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.ejml.simple.SimpleMatrix;
class HermiteSplineHelper {
    private HermiteSplineHelper() {

    }
    /**
     * Creates the basis weight matrix for a cubic hermite spline.
     *
     * @param start        The position of the spline at t = 0.
     * @param startTangent The tangent of the spline at t = 0.
     * @param end          The position of the spline at t = 1.
     * @param endTangent   The tangent of the spline at t = 1.
     * @return The basis weights for the spline.
     */
    public static SimpleMatrix createBasisWeightMatrix(Translation2d start, Translation2d startTangent,
                                                       Translation2d end, Translation2d endTangent) {
        // The basis weight matrix for hermite cubic splines is the following:
        // [x0  y0 ]
        // [x1  y1 ]
        // [dx0 dy0]
        // [dx1 dy1]
        return new SimpleMatrix(new double[][]{
                new double[]{start.getX(), start.getY()},
                new double[]{end.getX(), end.getY()},
                new double[]{startTangent.getX(), startTangent.getY()},
                new double[]{endTangent.getX(), endTangent.getY()}
        });
    }

    /**
     * Creates the basis weight matrix for a cubic hermite spline.
     *
     * @param start        The position of the spline at t = 0.
     * @param startHeading The heading of the spline at t = 0.
     * @param end          The position of the spline at t = 1.
     * @param endHeading   The heading of the spline at t = 1.
     * @return The basis weights for the spline.
     */
    public static SimpleMatrix createBasisWeightMatrix(Translation2d start, Rotation2d startHeading,
                                                       Translation2d end, Rotation2d endHeading) {
        double scale = 2.0 * end.minus(start).getNorm();

        return createBasisWeightMatrix(
                start, new Translation2d(startHeading.getCos(), startHeading.getSin()).times(scale),
                end, new Translation2d(endHeading.getCos(), endHeading.getSin()).times(scale)
        );
    }
}
