package org.frcteam2910.common.control;

public final class CentripetalAccelerationConstraint implements ITrajectoryConstraint {
    private final double maxCentripetalAcceleration;

    public CentripetalAccelerationConstraint(double maxCentripetalAcceleration) {
        this.maxCentripetalAcceleration = maxCentripetalAcceleration;
    }

    @Override
    public double getMaxVelocity(PathSegment segment) {
        return Math.sqrt(Math.abs(maxCentripetalAcceleration / segment.getCurvature()));
    }

    @Override
    public double getMaxAcceleration(PathSegment segment, double velocity) {
        return Double.POSITIVE_INFINITY;
    }
}
