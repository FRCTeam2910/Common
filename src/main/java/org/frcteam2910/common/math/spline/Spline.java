package org.frcteam2910.common.math.spline;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public abstract class Spline {
    public abstract Spline derivative();

    public abstract Vector2 getPoint(double t);

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
