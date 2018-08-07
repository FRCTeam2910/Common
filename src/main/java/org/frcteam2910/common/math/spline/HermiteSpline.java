package org.frcteam2910.common.math.spline;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public final class HermiteSpline extends Spline {
    private final double[] xCoefficients;
    private final double[] yCoefficients;

    public HermiteSpline(double[] xCoefficients, double[] yCoefficients) {
        this.xCoefficients = xCoefficients;
        this.yCoefficients = yCoefficients;

        assert xCoefficients.length == yCoefficients.length;
    }

    public static HermiteSpline cubic(RigidTransform2 start, RigidTransform2 end) {
        double[] xCoefficients = new double[4];
        double[] yCoefficients = new double[4];

        double scale = 2 * end.translation.subtract(start.translation).length;

        double x0 = start.translation.x;
        double x1 = end.translation.x;
        double dx0 = start.rotation.cos * scale;
        double dx1 = end.rotation.cos * scale;
        double y0 = start.translation.y;
        double y1 = end.translation.y;
        double dy0 = start.rotation.sin * scale;
        double dy1 = end.rotation.sin * scale;

        xCoefficients[0] = dx0 + dx1 + 2 * x0 - 2 * x1;
        xCoefficients[1] = -2 * dx0 - dx1 - 3 * x0 + 3 * x1;
        xCoefficients[2] = dx0;
        xCoefficients[3] = x0;

        yCoefficients[0] = dy0 + dy1 + 2 * y0 - 2 * y1;
        yCoefficients[1] = -2 * dy0 - dy1 - 3 * y0 + 3 * y1;
        yCoefficients[2] = dy0;
        yCoefficients[3] = y0;

        return new HermiteSpline(xCoefficients, yCoefficients);
    }

    public static HermiteSpline quintic(RigidTransform2 start, RigidTransform2 end) {
        double[] xCoefficients = new double[6];
        double[] yCoefficients = new double[6];

        double scale = 1.2 * end.translation.subtract(start.translation).length;

        double x0 = start.translation.x;
        double x1 = end.translation.x;
        double dx0 = start.rotation.cos * scale;
        double dx1 = end.rotation.cos * scale;
        double ddx0 = 0;
        double ddx1 = 0;
        double y0 = start.translation.y;
        double y1 = end.translation.y;
        double dy0 = start.rotation.sin * scale;
        double dy1 = end.rotation.sin * scale;
        double ddy0 = 0;
        double ddy1 = 0;

        xCoefficients[0] = -6 * x0 - 3 * dx0 - 0.5 * ddx0 + 0.5 * ddx1 - 3 * dx1 + 6 * x1;
        xCoefficients[1] = 15 * x0 + 8 * dx0 + 1.5 * ddx0 - ddx1 + 7 * dx1 - 15 * x1;
        xCoefficients[2] = -10 * x0 - 6 * dx0 - 1.5 * ddx0 + 0.5 * ddx1 - 4 * dx1 + 10 * x1;
        xCoefficients[3] = 0.5 * ddx0;
        xCoefficients[4] = dx0;
        xCoefficients[5] = x0;

        yCoefficients[0] = -6 * y0 - 3 * dy0 - 0.5 * ddy0 + 0.5 * ddy1 - 3 * dy1 + 6 * y1;
        yCoefficients[1] = 15 * y0 + 8 * dy0 + 1.5 * ddy0 - ddy1 + 7 * dy1 - 15 * y1;
        yCoefficients[2] = -10 * y0 - 6 * dy0 - 1.5 * ddy0 + 0.5 * ddy1 - 4 * dy1 + 10 * y1;
        yCoefficients[3] = 0.5 * ddy0;
        yCoefficients[4] = dy0;
        yCoefficients[5] = y0;
        
        return new HermiteSpline(xCoefficients, yCoefficients);
    }

    @Override
    public Vector2 getPoint(double t) {
        double x = 0.0;
        double y = 0.0;

        for (int i = 0; i < xCoefficients.length; i++) {
            x += Math.pow(t, xCoefficients.length - 1 - i) * xCoefficients[i];
            y += Math.pow(t, yCoefficients.length - 1 - i) * yCoefficients[i];
        }

        return new Vector2(x, y);
    }

    @Override
    public Rotation2 getHeading(double t) {
        double dx = 0.0;
        double dy = 0.0;

        for (int i = 0; i < xCoefficients.length - 1; i++) {
            dx += (xCoefficients.length - 1 - i) * Math.pow(t, xCoefficients.length - 2 - i) * xCoefficients[i];
            dy += (yCoefficients.length - 1 - i) * Math.pow(t, yCoefficients.length - 2 - i) * yCoefficients[i];
        }

        return new Rotation2(dx, dy, true);
    }
}
