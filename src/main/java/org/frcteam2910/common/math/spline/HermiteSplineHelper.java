package org.frcteam2910.common.math.spline;

import org.ejml.simple.SimpleMatrix;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

class HermiteSplineHelper {
    /**
     * Creates the basis weight matrix for a cubic hermite spline.
     *
     * @param start        The position of the spline at t = 0.
     * @param startTangent The tangent of the spline at t = 0.
     * @param end          The position of the spline at t = 1.
     * @param endTangent   The tangent of the spline at t = 1.
     * @return The basis weights for the spline.
     */
    public static SimpleMatrix createBasisWeightMatrix(Vector2 start, Vector2 startTangent,
                                                       Vector2 end, Vector2 endTangent) {
        // The basis weight matrix for hermite cubic splines is the following:
        // [x0  y0 ]
        // [x1  y1 ]
        // [dx0 dy0]
        // [dx1 dy1]
        return new SimpleMatrix(new double[][]{
                new double[]{start.x, start.y},
                new double[]{end.x, end.y},
                new double[]{startTangent.x, startTangent.y},
                new double[]{endTangent.x, endTangent.y}
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
    public static SimpleMatrix createBasisWeightMatrix(Vector2 start, Rotation2 startHeading,
                                                       Vector2 end, Rotation2 endHeading) {
        double scale = 2.0 * end.subtract(start).length;

        return createBasisWeightMatrix(
                start, Vector2.fromAngle(startHeading).scale(scale),
                end, Vector2.fromAngle(endHeading).scale(scale)
        );
    }
}
