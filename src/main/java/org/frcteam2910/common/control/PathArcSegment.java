package org.frcteam2910.common.control;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public final class PathArcSegment extends PathSegment {
    private final Vector2 center;
    private final Vector2 deltaStart;
    private final Vector2 deltaEnd;

    public PathArcSegment(Vector2 start, Vector2 end, Vector2 center) {
        super(start, end);
        this.center = center;
        this.deltaStart = start.subtract(center);
        this.deltaEnd = end.subtract(center);
    }

    /**
     * Constructs an arc from 3 points along an arc.
     *
     * @param a the point where the arc begins
     * @param b a point along the arc
     * @param c the point where the arc ends
     *
     * @return an arc going through the given points or null if an arc cannot be constructed out of the points
     */
    public static PathArcSegment fromPoints(Vector2 a, Vector2 b, Vector2 c) {
        PathLineSegment chordAB = new PathLineSegment(a, b);
        PathLineSegment chordBC = new PathLineSegment(b, c);

        RigidTransform2 perpChordAB = new RigidTransform2(chordAB.getPositionAtPercentage(0.5), chordAB.getHeadingAtPercentage(0.5).normal());
        RigidTransform2 perpChordBC = new RigidTransform2(chordBC.getPositionAtPercentage(0.5), chordBC.getHeadingAtPercentage(0.5).normal());

        Vector2 center = perpChordAB.intersection(perpChordBC);

        if (!Double.isFinite(center.x) || !Double.isFinite(center.y)) {
            return null;
        }

        // TODO: Check if the arc goes the long way around the circle.

        return new PathArcSegment(a, c, center);
    }

    @Override
    public PathSegment[] subdivide() {
        Vector2 mid = getPositionAtPercentage(0.5);
        return new PathSegment[] {
                new PathArcSegment(getStart(), mid, center),
                new PathArcSegment(mid, getEnd(), center)
        };
    }

    @Override
    public double getCurvature() {
        return 1.0 / deltaStart.length;
    }

    @Override
    public Vector2 getPositionAtPercentage(double percentage) {
        double deltaAngle = Vector2.getAngleBetween(deltaStart, deltaEnd).toRadians() *
                ((deltaStart.cross(deltaEnd) >= 0) ? 1 : -1) * percentage;
        return center.add(deltaStart.rotateBy(Rotation2.fromRadians(deltaAngle)));
    }

    @Override
    public Rotation2 getHeadingAtPercentage(double percentage) {
        double angle = Vector2.getAngleBetween(deltaStart, deltaEnd).toRadians() * percentage;
        return deltaStart.rotateBy(Rotation2.fromRadians(angle)).getAngle().normal();
    }

    public Vector2 getCenter() {
        return center;
    }

    @Override
    public double getLength() {
        return deltaStart.length * Vector2.getAngleBetween(deltaStart, deltaEnd).toRadians();
    }

    public double getRadius() {
        return deltaStart.length;
    }
}
