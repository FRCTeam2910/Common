package org.frcteam2910.common.control;

/**
 * An interface for a path generator.
 * <p>
 * An implementation may choose not to respect certain properties of the waypoints used to generate the path. However,
 * an implementation MUST respect the positions of the first and last waypoints.
 */
public interface PathGenerator {
    /**
     * Generates a path from a set of waypoints that can later be followed.
     *
     * @param waypoints the waypoints to generate the path from
     * @return          the path generated from the waypoints
     */
    Path generate(Waypoint... waypoints);
}
