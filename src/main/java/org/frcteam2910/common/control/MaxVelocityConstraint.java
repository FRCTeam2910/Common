package org.frcteam2910.common.control;

public final class MaxVelocityConstraint implements ITrajectoryConstraint {
    private final double velocity;

    public MaxVelocityConstraint(double velocity) {
        this.velocity = velocity;
    }

    @Override
    public double getMaxVelocity(PathSegment segment) {
        return velocity;
    }

    @Override
    public double getMaxAcceleration(PathSegment segment, double velocity) {
        return Double.POSITIVE_INFINITY;
    }
}
