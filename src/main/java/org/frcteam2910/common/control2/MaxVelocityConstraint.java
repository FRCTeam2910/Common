package org.frcteam2910.common.control2;

public class MaxVelocityConstraint implements ITrajectoryConstraint {
    private final double maxVelocity;

    public MaxVelocityConstraint(double maxVelocity) {
        this.maxVelocity = maxVelocity;
    }

    @Override
    public double getMaxVelocity(Path.State state) {
        return maxVelocity;
    }

    @Override
    public double getMaxAcceleration(Path.State state, double velocity) {
        return Double.POSITIVE_INFINITY;
    }

    @Override
    public double getMaxDeceleration(Path.State state, double velocity) {
        return Double.POSITIVE_INFINITY;
    }
}
