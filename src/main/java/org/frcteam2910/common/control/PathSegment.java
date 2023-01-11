package org.frcteam2910.common.control;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

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
        private final Translation2d position;
        private final Rotation2d heading;
        private final double curvature;

        public State(Translation2d position, Rotation2d heading, double curvature) {
            this.position = position;
            this.heading = heading;
            this.curvature = curvature;
        }

        public Translation2d getPosition() {
            return position;
        }

        public Rotation2d getHeading() {
            return heading;
        }

        public double getCurvature() {
            return curvature;
        }
    }
}
