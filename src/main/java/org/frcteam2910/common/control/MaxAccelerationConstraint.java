package org.frcteam2910.common.control;

public final class MaxAccelerationConstraint implements ITrajectoryConstraint {
    private final double maxAcceleration;

    public MaxAccelerationConstraint(double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
    }

    @Override
    public double getMaxVelocity(PathSegment segment) {
        return Double.POSITIVE_INFINITY;
    }

    @Override
    public double getMaxAcceleration(PathSegment segment, double velocity) {
        return maxAcceleration;
    }
}
