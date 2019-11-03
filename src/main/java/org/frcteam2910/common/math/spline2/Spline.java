package org.frcteam2910.common.math.spline2;

import org.ejml.simple.SimpleMatrix;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class Spline {
    private final SimpleMatrix basisMatrix;
    private final SimpleMatrix basisWeightMatrix;

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

    public Spline derivative() {
        SimpleMatrix coefficients = basisMatrix.mult(basisWeightMatrix);

        SimpleMatrix derivativeMatrix = new SimpleMatrix(coefficients.numRows() - 1, coefficients.numRows());
        // TODO: Fill out derivative matrix so that multiplying the coefficients by it results in a matrix containing
        // the coefficients of the derivative of the polynomial that the coefficients matrix represent
        for (int i = 0; i < derivativeMatrix.numRows(); i++) {
            derivativeMatrix.set(i, i, coefficients.numCols() - i);
        }

        return new Spline(SimpleMatrix.identity(getDegree()), derivativeMatrix.mult(coefficients));
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
