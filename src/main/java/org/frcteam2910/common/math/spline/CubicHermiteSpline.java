package org.frcteam2910.common.math.spline;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public final class CubicHermiteSpline extends Spline {
    private final double[] xCoefficients = new double[4];
    private final double[] yCoefficients = new double[4];

    public CubicHermiteSpline(RigidTransform2 start, RigidTransform2 end) {
        double x0, x1, dx0, dx1, y0, y1, dy0, dy1;
        double scale = 2 * end.translation.subtract(start.translation).length;
        x0 = start.translation.x;
        x1 = end.translation.x;
        dx0 = start.rotation.cos * scale;
        dx1 = end.rotation.cos * scale;
        y0 = start.translation.y;
        y1 = end.translation.y;
        dy0 = start.rotation.sin * scale;
        dy1 = end.rotation.sin * scale;
        xCoefficients[0] = dx0 + dx1 + 2 * x0 - 2 * x1;
        xCoefficients[1] = -2 * dx0 - dx1 - 3 * x0 + 3 * x1;
        xCoefficients[2] = dx0;
        xCoefficients[3] = x0;
        yCoefficients[0] = dy0 + dy1 + 2 * y0 - 2 * y1;
        yCoefficients[1] = -2 * dy0 - dy1 - 3 * y0 + 3 * y1;
        yCoefficients[2] = dy0;
        yCoefficients[3] = y0;
    }

    @Override
    public Vector2 getPoint(double t) {
        double x = 0.0;
        double y = 0.0;

        for (int i = 0; i < 4; i++) {
            x += Math.pow(t, 3 - i) * xCoefficients[i];
            y += Math.pow(t, 3 - i) * yCoefficients[i];
        }

        return new Vector2(x, y);
    }

    @Override
    public Rotation2 getHeading(double t) {
        double dx = 0.0;
        double dy = 0.0;

        for (int i = 0; i < 3; i++) {
            dx += (3 - i) * Math.pow(t, 2 - i) * xCoefficients[i];
            dy += (3 - i) * Math.pow(t, 2 - i) * yCoefficients[i];
        }

        return new Rotation2(dx, dy, true);
    }
}
