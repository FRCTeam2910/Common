package org.frcteam2910.common.control;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public final class PathLineSegment extends PathSegment {
    private final Vector2 delta;

    public PathLineSegment(Vector2 start, Vector2 end) {
        super(start, end);
        this.delta = end.subtract(start);
    }

    @Override
    public PathSegment[] subdivide() {
        Vector2 mid = getPositionAtPercentage(0.5);
        return new PathSegment[] {
                new PathLineSegment(getStart(), mid),
                new PathLineSegment(mid, getEnd())
        };
    }

    @Override
    public Vector2 getPositionAtPercentage(double percentage) {
        return getStart().add(delta.scale(percentage));
    }

    @Override
    public Rotation2 getHeadingAtPercentage(double percentage) {
        return delta.getAngle();
    }

    @Override
    public double getLength() {
        return delta.length;
    }
}
