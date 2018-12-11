package org.frcteam2910.common.control;

/**
 * A constraint that limits the acceleration by capping it at a maximum.
 * <p>
 * Usually this is used to make sure the robot won't tip over as we accelerate.
 */
public final class MaxAccelerationConstraint implements ITrajectoryConstraint {
    private final double maxAcceleration;

    /**
     * @param maxAcceleration the maximum linear acceleration of the robot. Must be positive.
     */
    public MaxAccelerationConstraint(double maxAcceleration) {
        if (maxAcceleration < 0.0) {
            throw new IllegalArgumentException("Max acceleration must be positive");
        }

        this.maxAcceleration = maxAcceleration;
    }

    @Override
    public double getMaxVelocity(PathSegment segment) {
        return Double.POSITIVE_INFINITY; // We don't constrain velocity
    }

    @Override
    public double getMaxAcceleration(PathSegment segment, double velocity) {
        return maxAcceleration;
    }
}
