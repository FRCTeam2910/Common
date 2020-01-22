package org.frcteam2910.common.math.spline;

import org.ejml.simple.SimpleMatrix;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class Spline {
    private final SimpleMatrix basisMatrix;
    private final SimpleMatrix basisWeightMatrix;

    private Spline derivative;

    public Spline(SimpleMatrix basisMatrix, SimpleMatrix basisWeightMatrix) {
        if (basisMatrix.numRows() != basisMatrix.numCols()) {
            throw new IllegalArgumentException("The basis matrix must be a square matrix");
        }
        if (basisWeightMatrix.numRows() != basisMatrix.numCols()) {
            throw new IllegalArgumentException("The basis weight matrix must be able to be multiplied by the basis matrix");
        }
        if (basisWeightMatrix.numCols() != 2) {
            throw new IllegalArgumentException("The basis weight matrix must have 2 columns");
        }

        this.basisMatrix = basisMatrix;
        this.basisWeightMatrix = basisWeightMatrix;
    }

    public int getDegree() {
        return basisMatrix.numCols() - 1;
    }

    public SimpleMatrix getBasisMatrix() {
        return basisMatrix;
    }

    public SimpleMatrix getBasisWeightMatrix() {
        return basisWeightMatrix;
    }

    /**
     * Gets the derivative of the spline.
     *
     * @return The spline's derivative.
     */
    public Spline derivative() {
        // The derivative is used when calculating the tangent or curvature. Cache it so we don't have to calculate it
        // multiple times.
        if (derivative == null) {
            SimpleMatrix coefficients = basisMatrix.mult(basisWeightMatrix);
            SimpleMatrix derivativeMatrix = new SimpleMatrix(coefficients.numRows() - 1, coefficients.numRows());
            for (int i = 0; i < derivativeMatrix.numRows(); i++) {
                derivativeMatrix.set(i, i + 1, i + 1);
            }

            derivative = new Spline(SimpleMatrix.identity(getDegree()), derivativeMatrix.mult(coefficients));
        }

        return derivative;
    }

    public Vector2 getPoint(double t) {
        SimpleMatrix result = SplineHelper.createPowerMatrix(getDegree(), t).mult(basisMatrix).mult(basisWeightMatrix);

        return new Vector2(result.get(0), result.get(1));
    }

    public Rotation2 getHeading(double t) {
        return derivative().getPoint(t).getAngle();
    }

    public double getCurvature(double t) {
        Spline d = derivative(); // 1st derivative
        Spline dd = d.derivative(); // 2nd derivative

        Vector2 dv = d.getPoint(t);
        Vector2 ddv = dd.getPoint(t);

        // Curvature can be calculated using the following equation:
        // k = (dv x ddv) / (dv . dv)^(3/2)
        //
        // https://en.wikipedia.org/wiki/Curvature#In_terms_of_a_general_parametrization
        return dv.cross(ddv) / (dv.dot(dv) * dv.length);
    }
}
