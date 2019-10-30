package org.frcteam2910.common.control;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.math.spline.HermiteSpline;
import org.frcteam2910.common.math.spline.Spline;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.BiFunction;

public class SplinePathBuilder {
    private List<SplineSegment> segmentList = new LinkedList<>();
    private Map<Double, Rotation2> rotationMap = new TreeMap<>();
    private double length = 0.0;

    private double splineLengthSampleStep = 1.0e-4;
    private BiFunction<RigidTransform2, RigidTransform2, Spline> hermiteSplineFactoryFunction = HermiteSpline::quintic;

    private PathSegment.State lastState;

    public SplinePathBuilder(Vector2 initialPosition, Rotation2 initialHeading, Rotation2 initialRotation) {
        lastState = new PathSegment.State(initialPosition, initialHeading, 0.0);
        rotationMap.put(0.0, initialHeading);
    }

    private void addSpline(Spline spline) {
        double splineLength = 0.0;
        Vector2 p0 = spline.getPoint(0.0);
        for (double t = splineLengthSampleStep; t <= 1.0; t += splineLengthSampleStep) {
            Vector2 p1 = spline.getPoint(t);
            splineLength += p1.subtract(p0).length;

            p0 = p1;
        }

        SplineSegment segment = new SplineSegment(spline, splineLength);
        segmentList.add(segment);
        lastState = segment.getEnd();
        length += splineLength;
    }

    public Path build() {
        return new Path(segmentList.toArray(new PathSegment[0]), rotationMap);
    }

    public SplinePathBuilder hermite(Vector2 position, Rotation2 heading) {
        addSpline(hermiteSplineFactoryFunction.apply(
                new RigidTransform2(lastState.getPosition(), lastState.getHeading()),
                new RigidTransform2(position, heading)
        ));
        return this;
    }

    public SplinePathBuilder hermite(Vector2 position, Rotation2 heading, Rotation2 rotation) {
        hermite(position, heading);
        rotationMap.put(length, rotation);
        return this;
    }

    public static class SplineSegment extends PathSegment {
        private final Spline spline;
        private final double length;

        private SplineSegment(Spline spline, double length) {
            this.spline = spline;
            this.length = length;
        }

        @Override
        public State calculate(double distance) {
            double t = distance / length;

            return new State(
                    spline.getPoint(t),
                    spline.getHeading(t),
                    spline.getCurvature(t)
            );
        }

        @Override
        public double getLength() {
            return length;
        }
    }
}
