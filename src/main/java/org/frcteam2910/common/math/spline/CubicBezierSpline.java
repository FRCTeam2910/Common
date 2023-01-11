package org.frcteam2910.common.math.spline;

import edu.wpi.first.math.geometry.Translation2d;
import org.ejml.simple.SimpleMatrix;

public final class CubicBezierSpline extends Spline {
    private static final SimpleMatrix BASIS_MATRIX = BezierSplineHelper.createBasisMatrix(3);
    private static final SimpleMatrix INVERSE_BASIS_MATRIX = BASIS_MATRIX.invert();

    public CubicBezierSpline(Translation2d start, Translation2d controlPoint1, Translation2d controlPoint2, Translation2d end) {
        super(BASIS_MATRIX, BezierSplineHelper.createBasisWeightMatrix(start, controlPoint1, controlPoint2, end));
    }

    private CubicBezierSpline(SimpleMatrix basisWeightMatrix) {
        super(BASIS_MATRIX, basisWeightMatrix);
    }

    /**
     * Converts a cubic spline to a cubic spline with a bezier representation.
     *
     * @param spline The spline to convert.
     * @return The bezier representation of the spline.
     */
    public static CubicBezierSpline convert(Spline spline) {
        if (spline.getDegree() != 3) {
            throw new IllegalArgumentException("Spline must be cubic.");
        }

        // B1 * W1 = B2 * W2
        // W1 = B1^-1 * B2 * W2
        return new CubicBezierSpline(INVERSE_BASIS_MATRIX.mult(spline.getBasisMatrix()).mult(spline.getBasisWeightMatrix()));
    }

    public Translation2d[] getControlPoints() {
        return BezierSplineHelper.basisWeightMatrixToControlPoints(getBasisWeightMatrix());
    }
}
