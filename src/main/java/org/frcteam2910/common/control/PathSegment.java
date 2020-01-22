package org.frcteam2910.common.control;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public abstract class PathSegment {
    public State getStart() {
        return calculate(0.0);
    }

    public State getEnd() {
        return calculate(getLength());
    }

    public abstract State calculate(double distance);

    public abstract double getLength();

    public static class State {
        private final Vector2 position;
        private final Rotation2 heading;
        private final double curvature;

        public State(Vector2 position, Rotation2 heading, double curvature) {
            this.position = position;
            this.heading = heading;
            this.curvature = curvature;
        }

        public Vector2 getPosition() {
            return position;
        }

        public Rotation2 getHeading() {
            return heading;
        }

        public double getCurvature() {
            return curvature;
        }
    }
}
