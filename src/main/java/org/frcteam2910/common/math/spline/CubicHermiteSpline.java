package org.frcteam2910.common.math.spline;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.ejml.simple.SimpleMatrix;

public class CubicHermiteSpline extends Spline {
    private static final SimpleMatrix BASIS_MATRIX = new SimpleMatrix(new double[][]{
            new double[]{1, 0, 0, 0},
            new double[]{0, 0, 1, 0},
            new double[]{-3, 3, -2, -1},
            new double[]{2, -2, 1, 1},
    });
    private static final SimpleMatrix INVERSE_BASIS_MATRIX = BASIS_MATRIX.invert();

    public CubicHermiteSpline(Translation2d start, Translation2d startTangent,
                              Translation2d end, Translation2d endTangent) {
        this(HermiteSplineHelper.createBasisWeightMatrix(start, startTangent, end, endTangent));
    }

    public CubicHermiteSpline(Translation2d start, Rotation2d startHeading,
                              Translation2d end, Rotation2d endHeading) {
        this(HermiteSplineHelper.createBasisWeightMatrix(start, startHeading, end, endHeading));
    }

    private CubicHermiteSpline(SimpleMatrix basisWeightMatrix) {
        super(BASIS_MATRIX, basisWeightMatrix);
    }

    /**
     * Converts a cubic spline to a cubic spline with a hermite representation.
     *
     * @param spline The spline to convert.
     * @return The hermite representation of the spline.
     */
    public static CubicHermiteSpline convert(Spline spline) {
        if (spline.getDegree() != 3) {
            throw new IllegalArgumentException("Spline must be cubic.");
        }

        // B1 * W1 = B2 * W2
        // W1 = B1^-1 * B2 * W2
        return new CubicHermiteSpline(INVERSE_BASIS_MATRIX.mult(spline.getBasisMatrix()).mult(spline.getBasisWeightMatrix()));
    }
}
