package org.frcteam2910.common.control2;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class Path {
    private final double[] distancesFromStart;
    private final PathSegment[] segments;

    private final double length;

    public Path(PathSegment[] segments) {
        this.segments = segments;

        distancesFromStart = new double[segments.length];
        double length = 0.0;
        for (int i = 0; i < segments.length; i++) {
            distancesFromStart[i] = length;
            length += segments[i].getLength();
        }
        this.length = length;
    }

    private double getDistanceToSegmentStart(int segment) {
        return distancesFromStart[segment];
    }

    private double getDistanceToSegmentEnd(int segment) {
        return distancesFromStart[segment] + segments[segment].getLength();
    }

    private int getSegmentAtDistance(double distance) {
        int start = 0;
        int end = segments.length - 1;
        int mid = start + (end - start) / 2;

        while (start <= end) {
            mid = (start + end) / 2;

            if (distance > getDistanceToSegmentEnd(mid)) {
                start = mid + 1;
            } else if (distance < getDistanceToSegmentStart(mid)) {
                end = mid - 1;
            } else {
                break;
            }
        }

        return mid;
    }

    public State calculate(double distance) {
        int currentSegment = getSegmentAtDistance(distance);
        PathSegment segment = segments[currentSegment];
        double segmentDistance = distance - getDistanceToSegmentStart(currentSegment);

        return segment.calculate(segmentDistance);
    }

    public double getLength() {
        return length;
    }

    public static class State {
        private final double distance;
        private final Vector2 translation;
        private final Rotation2 heading;
        private final Rotation2 rotation;
        private final double curvature;

        public State(double distance, Vector2 translation, Rotation2 heading, Rotation2 rotation, double curvature) {
            this.distance = distance;
            this.translation = translation;
            this.heading = heading;
            this.rotation = rotation;
            this.curvature = curvature;
        }

        public double getDistance() {
            return distance;
        }

        public Vector2 getTranslation() {
            return translation;
        }

        public Rotation2 getHeading() {
            return heading;
        }

        public Rotation2 getRotation() {
            return rotation;
        }

        public double getCurvature() {
            return curvature;
        }
    }
}
