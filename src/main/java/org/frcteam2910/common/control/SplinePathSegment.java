package org.frcteam2910.common.control;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.math.spline.Spline;

public class SplinePathSegment extends PathSegment {
    private static final double SPLINE_LENGTH_DT = 1.0e-3;

    private final Spline spline;
    private final Rotation2 startingRotation;
    private final Rotation2 endingRotation;

    private final double length;

    public SplinePathSegment(Spline spline, Rotation2 startingRotation, Rotation2 endingRotation) {
        this.spline = spline;
        this.startingRotation = startingRotation;
        this.endingRotation = endingRotation;

        double length = 0.0;
        Vector2 p0 = spline.getPoint(0.0);
        for (double t = SPLINE_LENGTH_DT; t <= 1.0; t += SPLINE_LENGTH_DT) {
            Vector2 p1 = spline.getPoint(t);
            length += p1.subtract(p0).length;

            p0 = p1;
        }

        this.length = length;
    }

    @Override
    public Path.State calculate(double distance) {
        double t = distance / length;

        return new Path.State(
                distance,
                spline.getPoint(t),
                spline.getHeading(t),
                startingRotation.interpolate(endingRotation, t),
                spline.getCurvature(t)
        );
    }

    @Override
    public double getLength() {
        return length;
    }
}
