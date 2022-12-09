package org.frcteam2910.common.control;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.frcteam2910.common.util.Angles;

import java.util.ArrayList;
import java.util.List;
import java.util.TreeMap;

public final class SimplePathBuilder {
    private List<PathSegment> segmentList = new ArrayList<>();
    private TreeMap<Double, Rotation2d> rotationMap = new TreeMap<>();

    private Translation2d lastPosition;
    private double length = 0.0;

    public SimplePathBuilder(Translation2d initialPosition, Rotation2d initialRotation) {
        this.lastPosition = initialPosition;

        rotationMap.put(0.0, initialRotation);
    }

    private void addSegment(PathSegment segment) {
        segmentList.add(segment);
        length += segment.getLength();
        lastPosition = segment.getEnd().getPosition();
    }

    private void addSegment(PathSegment segment, Rotation2d rotation) {
        addSegment(segment);
        rotationMap.put(length, rotation);
    }

    public Path build() {
        return new Path(segmentList.toArray(new PathSegment[0]), rotationMap);
    }

    public SimplePathBuilder arcTo(Translation2d position, Translation2d center) {
        addSegment(new ArcSegment(lastPosition, position, center));
        return this;
    }

    public SimplePathBuilder arcTo(Translation2d position, Translation2d center, Rotation2d rotation) {
        addSegment(new ArcSegment(lastPosition, position, center), rotation);
        return this;
    }

    public SimplePathBuilder lineTo(Translation2d position) {
        addSegment(new LineSegment(lastPosition, position));
        return this;
    }

    public SimplePathBuilder lineTo(Translation2d position, Rotation2d rotation) {
        addSegment(new LineSegment(lastPosition, position), rotation);
        return this;
    }

    public static final class ArcSegment extends PathSegment {
        private final Translation2d center;
        private final Translation2d deltaStart;
        private final Translation2d deltaEnd;
        private final boolean clockwise;
        private final Rotation2d arcAngle;

        private final double curvature;

        private  final double length;

        public ArcSegment(Translation2d start, Translation2d end, Translation2d center) {
            this.center = center;
            deltaStart = start.minus(center);
            deltaEnd = end.minus(center);

            var cross = deltaStart.getX() * deltaEnd.getY() - deltaStart.getY() * deltaEnd.getX();
            clockwise = cross <= 0.0;

            var r1 = new Rotation2d(deltaStart.getX(), deltaStart.getY());
            var r2 = new Rotation2d(deltaEnd.getX(), deltaEnd.getY());

            arcAngle = Rotation2d.fromDegrees(
                    Math.toDegrees(Angles.shortestAngularDistance(r1.getRadians(), r2.getRadians())));

            curvature = 1.0 / deltaStart.getNorm();
            length = deltaStart.getNorm() * arcAngle.getRadians();
        }

        @Override
        public State calculate(double distance) {
            double percentage = distance / length;

            Translation2d sampleHeading = deltaStart.rotateBy(Rotation2d.fromDegrees(percentage + (clockwise ? -1.0 : 1.0) * 90));
            Rotation2d newHeading = new Rotation2d(sampleHeading.getX(), sampleHeading.getY());

            return new State(
                    center.plus(deltaStart.rotateBy(Rotation2d.fromDegrees(percentage))),
                    newHeading,
                    curvature
            );
        }

        @Override
        public double getLength() {
            return length; //deltaStart.length * Vector2.getAngleBetween(deltaStart, deltaEnd).toRadians();
        }
    }

    public static final class LineSegment extends PathSegment {
        private final Translation2d start;
        private final Translation2d delta;
        private final Rotation2d heading;

        private LineSegment(Translation2d start, Translation2d end) {
            this.start = start;
            this.delta = end.minus(start);
            this.heading = new Rotation2d(delta.getX(), delta.getY());
        }

        @Override
        public State calculate(double distance) {
            return new State(
                    start.plus(delta.times(distance / getLength())),
                    heading,
                    0.0
            );
        }

        @Override
        public double getLength() {
            return delta.getNorm();
        }
    }
}
