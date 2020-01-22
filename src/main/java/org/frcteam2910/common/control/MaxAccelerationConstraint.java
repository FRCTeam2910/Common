package org.frcteam2910.common.control;

/**
 * A constraint that limits the acceleration.
 */
public class MaxAccelerationConstraint extends TrajectoryConstraint {
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
    public double getMaxAcceleration(Path.State state, double velocity) {
        return maxAcceleration;
    }

    @Override
    public double getMaxDeceleration(Path.State state, double velocity) {
        return maxDeceleration;
    }
}
