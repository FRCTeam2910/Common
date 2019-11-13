package org.frcteam2910.common.control;

import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.math.spline.Spline;

public final class SplinePathSegment extends PathSegment {
    private static final double LENGTH_SAMPLE_STEP = 1.0e-4;

    private final Spline spline;

    private transient double length = Double.NaN;

    public SplinePathSegment(Spline spline) {
        this.spline = spline;
    }

    @Override
    public State calculate(double distance) {
        double t = distance / getLength();

        return new State(
                spline.getPoint(t),
                spline.getHeading(t),
                spline.getCurvature(t)
        );
    }

    @Override
    public double getLength() {
        if (!Double.isFinite(length)) {
            length = 0.0;
            Vector2 p0 = spline.getPoint(0.0);
            for (double t = LENGTH_SAMPLE_STEP; t <= 1.0; t += LENGTH_SAMPLE_STEP) {
                Vector2 p1 = spline.getPoint(t);
                length += p1.subtract(p0).length;

                p0 = p1;
            }
        }

        return length;
    }

    public Spline getSpline() {
        return spline;
    }
}
