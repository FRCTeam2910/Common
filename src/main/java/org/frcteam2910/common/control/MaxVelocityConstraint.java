package org.frcteam2910.common.control;

public final class MaxVelocityConstraint implements ITrajectoryConstraint {
    private final double maxVelocity;

    public MaxVelocityConstraint(double maxVelocity) {
        this.maxVelocity = maxVelocity;
    }

    @Override
    public double getMaxVelocity(PathSegment segment) {
        return maxVelocity;
    }

    @Override
    public double getMaxAcceleration(PathSegment segment, double velocity) {
        return Double.POSITIVE_INFINITY;
    }
}
