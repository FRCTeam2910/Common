package org.frcteam2910.common.control;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.TreeMap;

public final class SimplePathBuilder {
    private List<PathSegment> segmentList = new ArrayList<>();
    private TreeMap<Double, Rotation2> rotationMap = new TreeMap<>();

    private Vector2 lastPosition;
    private double length = 0.0;

    public SimplePathBuilder(Vector2 initialPosition, Rotation2 initialRotation) {
        this.lastPosition = initialPosition;

        rotationMap.put(0.0, initialRotation);
    }

    private void addSegment(PathSegment segment) {
        segmentList.add(segment);
        length += segment.getLength();
        lastPosition = segment.getEnd().getPosition();
    }

    private void addSegment(PathSegment segment, Rotation2 rotation) {
        addSegment(segment);
        rotationMap.put(length, rotation);
    }

    public Path build() {
        return new Path(segmentList.toArray(new PathSegment[0]), rotationMap);
    }

    public SimplePathBuilder arcTo(Vector2 position, Vector2 center) {
        addSegment(new ArcSegment(lastPosition, position, center));
        return this;
    }

    public SimplePathBuilder arcTo(Vector2 position, Vector2 center, Rotation2 rotation) {
        addSegment(new ArcSegment(lastPosition, position, center), rotation);
        return this;
    }

    public SimplePathBuilder lineTo(Vector2 position) {
        addSegment(new LineSegment(lastPosition, position));
        return this;
    }

    public SimplePathBuilder lineTo(Vector2 position, Rotation2 rotation) {
        addSegment(new LineSegment(lastPosition, position), rotation);
        return this;
    }

    public static final class ArcSegment extends PathSegment {
        private final Vector2 center;
        private final Vector2 deltaStart;
        private final Vector2 deltaEnd;
        private final boolean clockwise;

        public ArcSegment(Vector2 start, Vector2 end, Vector2 center) {
            this.center = center;
            this.deltaStart = start.subtract(center);
            this.deltaEnd = end.subtract(center);

            clockwise = deltaStart.cross(deltaEnd) <= 0.0;
        }

        @Override
        public State calculate(double distance) {
            double percentage = distance / getLength();

            double angle = Vector2.getAngleBetween(deltaStart, deltaEnd).toRadians() *
                    (clockwise ? -1.0 : 1.0) * percentage;
            return new State(
                    center.add(deltaStart.rotateBy(Rotation2.fromRadians(angle))),
                    // TODO: Use cross product instead of just adding 90deg when calculating heading
                    deltaStart.rotateBy(Rotation2.fromRadians(angle + (clockwise ? -1.0 : 1.0) * 0.5 * Math.PI)).getAngle(),
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

        private LineSegment(Vector2 start, Vector2 end) {
            this.start = start;
            this.delta = end.subtract(start);
        }

        @Override
        public State calculate(double distance) {
            return new State(
                    start.add(delta.scale(distance / getLength())),
                    delta.getAngle(),
                    0.0
            );
        }

        @Override
        public double getLength() {
            return delta.length;
        }
    }
}
