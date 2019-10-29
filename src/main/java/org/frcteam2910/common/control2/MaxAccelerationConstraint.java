package org.frcteam2910.common.control2;

public class MaxAccelerationConstraint implements ITrajectoryConstraint {
    private final double maxAcceleration;
    private final double maxDeceleration;

    public MaxAccelerationConstraint(double maxAbsAcceleration) {
        this(maxAbsAcceleration, maxAbsAcceleration);
    }

    public MaxAccelerationConstraint(double maxAcceleration, double maxDeceleration) {
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
    }

    @Override
    public double getMaxVelocity(Path.State state) {
        return Double.POSITIVE_INFINITY;
    }

    @Override
    public double getMaxAcceleration(Path.State state, double velocity) {
        return maxAcceleration;
    }

    @Override
    public double getMaxDeceleration(Path.State state, double velocity) {
        return maxDeceleration;
    }
}
