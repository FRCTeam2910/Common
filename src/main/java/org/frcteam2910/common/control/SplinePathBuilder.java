package org.frcteam2910.common.control;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.math.spline.HermiteSpline;
import org.frcteam2910.common.math.spline.Spline;

import java.util.ArrayList;
import java.util.List;

public class SplinePathBuilder {
    private List<Waypoint> waypointList = new ArrayList<>();

    public SplinePathBuilder addWaypoint(Waypoint waypoint) {
        waypointList.add(waypoint);
        return this;
    }

    public SplinePathBuilder addWaypoint(Vector2 position, Rotation2 heading) {
        return addWaypoint(new Waypoint(position, heading));
    }

    public SplinePathBuilder addWaypoint(Vector2 position, Rotation2 heading, Rotation2 rotation) {
        return addWaypoint(new Waypoint(position, heading, rotation));
    }

    public SplinePathBuilder addWaypoints(Waypoint... waypoints) {
        for (Waypoint waypoint : waypoints) {
            addWaypoint(waypoint);
        }

        return this;
    }

    public Path build() {
        if (waypointList.size() < 2) {
            throw new IllegalStateException("At least 2 waypoints must be added before the path is built.");
        }

        SplineSegment[] segments = new SplineSegment[waypointList.size() - 1];
        for (int i = 0; i < segments.length; i++) {
            Waypoint start = waypointList.get(i);
            Waypoint end = waypointList.get(i + 1);

            segments[i] = new SplineSegment(HermiteSpline.quintic(
                    new RigidTransform2(start.position, start.heading),
                    new RigidTransform2(end.position, end.heading)),
                    start.rotation,
                    end.rotation
            );
        }

        return new Path(segments);
    }

    public static class SplineSegment extends PathSegment {
        private static final double SPLINE_LENGTH_DT = 1.0e-3;

        private final Spline spline;
        private final Rotation2 startingRotation;
        private final Rotation2 endingRotation;

        private final double length;

        public SplineSegment(Spline spline, Rotation2 startingRotation, Rotation2 endingRotation) {
            this.spline = spline;
            this.startingRotation = startingRotation;
            this.endingRotation = endingRotation;

            double length = 0.0;
            Vector2 p0 = spline.getPoint(0.0);
            for (double t = SPLINE_LENGTH_DT; t <= 1.0; t += SPLINE_LENGTH_DT) {
                Vector2 p1 = spline.getPoint(t);
                length += p1.subtract(p0).length;

                p0 = p1;
            }

            this.length = length;
        }

        @Override
        public Path.State calculate(double distance) {
            double t = distance / length;

            return new Path.State(
                    distance,
                    spline.getPoint(t),
                    spline.getHeading(t),
                    startingRotation.interpolate(endingRotation, t),
                    spline.getCurvature(t)
            );
        }

        @Override
        public double getLength() {
            return length;
        }
    }
}
