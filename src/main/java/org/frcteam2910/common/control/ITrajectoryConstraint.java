package org.frcteam2910.common.control;

public interface ITrajectoryConstraint {
    /**
     * Get the maximum velocity this constraint allows for this segment.
     *
     * @param segment the segment
     * @return the maximum velocity
     */
    double getMaxVelocity(PathSegment segment);

    /**
     * Get the maximum acceleration this constraint allows for this segment at a velocity.
     *
     * @param segment  the segment
     * @param velocity the velocity
     * @return the maximum acceleration
     */
    double getMaxAcceleration(PathSegment segment, double velocity);
}
