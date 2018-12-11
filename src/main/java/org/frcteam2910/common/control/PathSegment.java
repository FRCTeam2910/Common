package org.frcteam2910.common.control;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public abstract class PathSegment {
    private final Vector2 start;
    private final Vector2 end;

    public PathSegment(Vector2 start, Vector2 end) {
        this.start = start;
        this.end = end;
    }

    /**
     * Gets the curvature of the segment. Curvature can be thought of the rate of change of the heading.
     *
     * @return the segment's curvature
     */
    public abstract double getCurvature();

    public abstract PathSegment[] subdivide();

    public Vector2 getStart() {
        return start;
    }

    public Vector2 getEnd() {
        return end;
    }

    public Vector2 getPositionAtDistance(double distance) {
        return getPositionAtPercentage(distance / getLength());
    }

    public abstract Vector2 getPositionAtPercentage(double percentage);

    public Rotation2 getHeadingAtDistance(double distance) {
        return getHeadingAtPercentage(distance / getLength());
    }

    public abstract Rotation2 getHeadingAtPercentage(double percentage);

    public abstract double getLength();
}
