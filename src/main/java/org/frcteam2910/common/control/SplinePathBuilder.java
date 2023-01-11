package org.frcteam2910.common.control;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.frcteam2910.common.math.spline.CubicBezierSpline;
import org.frcteam2910.common.math.spline.CubicHermiteSpline;
import org.frcteam2910.common.math.spline.Spline;

import java.util.*;

public final class SplinePathBuilder {
    private List<PathSegment> segmentList = new ArrayList<>();
    private Map<Double, Rotation2d> rotationMap = new TreeMap<>();
    private double length = 0.0;

    private PathSegment.State lastState;

    public SplinePathBuilder(Translation2d initialPosition, Rotation2d initialHeading, Rotation2d initialRotation) {
        lastState = new PathSegment.State(initialPosition, initialHeading, 0.0);
        rotationMap.put(0.0, initialRotation);
    }

    private void addSpline(Spline spline) {
        SplinePathSegment segment = new SplinePathSegment(spline);
        segmentList.add(segment);
        lastState = segment.getEnd();
        length += segment.getLength();
    }

    public Path build() {
        return new Path(segmentList.toArray(new PathSegment[0]), rotationMap);
    }

    public SplinePathBuilder bezier(Translation2d controlPoint1, Translation2d controlPoint2, Translation2d end) {
        addSpline(new CubicBezierSpline(
                lastState.getPosition(),
                controlPoint1,
                controlPoint2,
                end
        ));
        return this;
    }

    public SplinePathBuilder bezier(Translation2d controlPoint1, Translation2d controlPoint2, Translation2d end, Rotation2d rotation) {
        bezier(controlPoint1, controlPoint2, end);
        rotationMap.put(length, rotation);
        return this;
    }

    public SplinePathBuilder hermite(Translation2d position, Rotation2d heading) {
        addSpline(new CubicHermiteSpline(
                lastState.getPosition(), lastState.getHeading(),
                position, heading
        ));
        return this;
    }

    public SplinePathBuilder hermite(Translation2d position, Rotation2d heading, Rotation2d rotation) {
        hermite(position, heading);
        rotationMap.put(length, rotation);
        return this;
    }
}
