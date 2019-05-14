package org.frcteam2910.common.control;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.io.Serializable;
import java.util.Objects;

public final class PathArcSegment extends PathSegment implements Serializable {
    private static final long serialVersionUID = 1817031344401689498L;

    private final Vector2 center;
    private final Vector2 deltaStart;
    private final Vector2 deltaEnd;
    private final boolean clockwise;

    public PathArcSegment(Vector2 start, Vector2 end, Vector2 center) {
        super(start, end);
        this.center = center;
        this.deltaStart = start.subtract(center);
        this.deltaEnd = end.subtract(center);

        clockwise = deltaStart.cross(deltaEnd) <= 0.0;
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
    public PathArcSegment[] subdivide() {
        Vector2 mid = getPositionAtPercentage(0.5);
        return new PathArcSegment[] {
                new PathArcSegment(getStart(), mid, center),
                new PathArcSegment(mid, getEnd(), center)
        };
    }

    @Override
    public PathArcSegment mirror() {
        return new PathArcSegment(
                getStart().multiply(1.0, -1.0),
                getEnd().multiply(1.0, -1.0),
                center.multiply(1.0, -1.0)
        );
    }

    @Override
    public double getCurvature() {
        return 1.0 / deltaStart.length;
    }

    @Override
    public Vector2 getPositionAtPercentage(double percentage) {
        double deltaAngle = Vector2.getAngleBetween(deltaStart, deltaEnd).toRadians() *
                (clockwise ? -1.0 : 1.0) * percentage;
        return center.add(deltaStart.rotateBy(Rotation2.fromRadians(deltaAngle)));
    }

    @Override
    public Rotation2 getHeadingAtPercentage(double percentage) {
        double angle = Vector2.getAngleBetween(deltaStart, deltaEnd).toRadians() *
                (clockwise ? -1.0 : 1.0) * percentage +
                (clockwise ? -0.5 * Math.PI : 0.5 * Math.PI); // Add or subtract 90 degrees to the angle based on the direction of travel
        return deltaStart.rotateBy(Rotation2.fromRadians(angle)).getAngle();
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

    @Override
    public boolean equals(Object o) {
        if (o instanceof PathArcSegment) {
            PathArcSegment other = (PathArcSegment) o;
            return deltaStart.equals(other.deltaStart) && deltaEnd.equals(other.deltaEnd) && center.equals(other.center);
        }

        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(center, deltaStart, deltaEnd);
    }

    @Override
    public String toString() {
        return String.format("{start: %s, end: %s, center: %s}", getStart(), getEnd(), center);
    }
}
