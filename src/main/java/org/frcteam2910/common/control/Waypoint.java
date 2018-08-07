package org.frcteam2910.common.control;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class Waypoint {
    public final Vector2 position;
    public final Rotation2 heading;
    public final Rotation2 rotation;

    public Waypoint(RigidTransform2 pose) {
        this(pose.translation, pose.rotation);
    }

    public Waypoint(Vector2 position, Rotation2 heading) {
        this.position = position;
        this.heading = heading;
        this.rotation = null;
    }

    public Waypoint(RigidTransform2 pose, Rotation2 rotation) {
        this(pose.translation, pose.rotation, rotation);
    }

    public Waypoint(Vector2 position, Rotation2 heading, Rotation2 rotation) {
        this.position = position;
        this.heading = heading;
        this.rotation = rotation;
    }

    public Waypoint(Waypoint other) {
        this(other.position, other.heading, other.rotation);
    }
}
