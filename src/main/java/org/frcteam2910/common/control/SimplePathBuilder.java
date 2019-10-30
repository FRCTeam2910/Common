package org.frcteam2910.common.control;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.util.LinkedList;
import java.util.List;

public final class SimplePathBuilder {
    private List<PathSegment> segmentList = new LinkedList<>();
    private Vector2 lastPosition;
    private Rotation2 lastRotation;

    public SimplePathBuilder(Vector2 initialPosition, Rotation2 initialRotation) {
        this.lastPosition = initialPosition;
        this.lastRotation = initialRotation;
    }

    public Path build() {
        return new Path(segmentList.toArray(new PathSegment[0]));
    }

    public SimplePathBuilder arcTo(Vector2 position, Vector2 center, Rotation2 rotation) {
        segmentList.add(new ArcSegment(lastPosition, position, center, lastRotation, rotation));
        lastPosition = position;
        lastRotation = rotation;

        return this;
    }

    public SimplePathBuilder lineTo(Vector2 position, Rotation2 rotation) {
        segmentList.add(new LineSegment(lastPosition, position, lastRotation, rotation));
        lastPosition = position;
        lastRotation = rotation;

        return this;
    }

    public static final class ArcSegment extends PathSegment {
        private final Vector2 center;
        private final Vector2 deltaStart;
        private final Vector2 deltaEnd;
        private final boolean clockwise;

        private final Rotation2 startingRotation;
        private final Rotation2 endingRotation;

        public ArcSegment(Vector2 start, Vector2 end, Vector2 center, Rotation2 startingRotation, Rotation2 endingRotation) {
            this.center = center;
            this.deltaStart = start.subtract(center);
            this.deltaEnd = end.subtract(center);
            this.startingRotation = startingRotation;
            this.endingRotation = endingRotation;

            clockwise = deltaStart.cross(deltaEnd) <= 0.0;
        }

        @Override
        public Path.State calculate(double distance) {
            double percentage = distance / getLength();

            double angle = Vector2.getAngleBetween(deltaStart, deltaEnd).toRadians() *
                    (clockwise ? -1.0 : 1.0) * percentage;
            return new Path.State(
                    distance,
                    center.add(deltaStart.rotateBy(Rotation2.fromRadians(angle))),
                    // TODO: Use cross product instead of just adding 90deg when calculating heading
                    deltaStart.rotateBy(Rotation2.fromRadians(angle + (clockwise ? -1.0 : 1.0) * 0.5 * Math.PI)).getAngle(),
                    startingRotation.interpolate(endingRotation, percentage),
                    1.0 / deltaStart.length
            );
        }

        @Override
        public double getLength() {
            return deltaStart.length * Vector2.getAngleBetween(deltaStart, deltaEnd).toRadians();
        }
    }

    public static final class LineSegment extends PathSegment {
        private final Vector2 start;
        private final Vector2 delta;
        private final Rotation2 startingRotation;
        private final Rotation2 endingRotation;

        private LineSegment(Vector2 start, Vector2 end, Rotation2 startingRotation, Rotation2 endingRotation) {
            this.start = start;
            this.delta = end.subtract(start);
            this.startingRotation = startingRotation;
            this.endingRotation = endingRotation;
        }

        @Override
        public Path.State calculate(double distance) {
            return new Path.State(
                    distance,
                    start.add(delta.scale(distance / getLength())),
                    delta.getAngle(),
                    startingRotation.interpolate(endingRotation, distance / getLength()),
                    0.0
            );
        }

        @Override
        public double getLength() {
            return delta.length;
        }
    }
}
