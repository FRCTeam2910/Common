package org.frcteam2910.common.control;

public interface ITrajectoryConstraint {
    double getMaxVelocity(PathSegment segment);

    double getMaxAcceleration(PathSegment segment, double velocity);
}
