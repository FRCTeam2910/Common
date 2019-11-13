package org.frcteam2910.common.control;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;

import java.util.Map;

public class Path {
    private final PathSegment[] segments;
    private final InterpolatingTreeMap<InterpolatingDouble, Rotation2> rotationMap = new InterpolatingTreeMap<>();
    private final double[] distancesFromStart;

    private final double length;

    public Path(PathSegment[] segments, Map<Double, Rotation2> rotationMap) {
        this.segments = segments;

        for (Map.Entry<Double, Rotation2> rotationEntry : rotationMap.entrySet()) {
            this.rotationMap.put(new InterpolatingDouble(rotationEntry.getKey()), rotationEntry.getValue());
        }

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

        PathSegment.State state = segment.calculate(segmentDistance);

        return new Path.State(
                distance,
                state.getPosition(),
                state.getHeading(),
                rotationMap.getInterpolated(new InterpolatingDouble(distance)),
                state.getCurvature()
        );
    }

    public double getLength() {
        return length;
    }

    public PathSegment[] getSegments() {
        return segments;
    }

    public InterpolatingTreeMap<InterpolatingDouble, Rotation2> getRotationMap() {
        return rotationMap;
    }

    public static class State {
        private final double distance;
        private final Vector2 position;
        private final Rotation2 heading;
        private final Rotation2 rotation;
        private final double curvature;

        public State(double distance, Vector2 position, Rotation2 heading, Rotation2 rotation, double curvature) {
            this.distance = distance;
            this.position = position;
            this.heading = heading;
            this.rotation = rotation;
            this.curvature = curvature;
        }

        public double getDistance() {
            return distance;
        }

        public Vector2 getPosition() {
            return position;
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
