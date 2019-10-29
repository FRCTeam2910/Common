package org.frcteam2910.common.control2;

import org.frcteam2910.common.control.Waypoint;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.math.spline.HermiteSpline;

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

        SplinePathSegment[] segments = new SplinePathSegment[waypointList.size() - 1];
        for (int i = 0; i < segments.length; i++) {
            Waypoint start = waypointList.get(i);
            Waypoint end = waypointList.get(i + 1);

            segments[i] = new SplinePathSegment(HermiteSpline.quintic(
                    new RigidTransform2(start.position, start.heading),
                    new RigidTransform2(end.position, end.heading)),
                    start.rotation,
                    end.rotation
            );
        }

        return new Path(segments);
    }
}
