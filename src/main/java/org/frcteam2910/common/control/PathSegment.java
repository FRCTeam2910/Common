package org.frcteam2910.common.control;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.io.Serializable;

public abstract class PathSegment implements Serializable {
    private static final long serialVersionUID = -2218546434968558348L;

    private final Vector2 start;
    private final Vector2 end;

    public PathSegment(Vector2 start, Vector2 end) {
        this.start = start;
        this.end = end;
    }

    public abstract PathSegment[] subdivide();

    public abstract PathSegment mirror();

    /**
     * Gets the curvature of the segment.
     *
     * @return the curvature of the segment
     */
    public abstract double getCurvature();

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
