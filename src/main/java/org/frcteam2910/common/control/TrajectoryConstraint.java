package org.frcteam2910.common.control;

public abstract class TrajectoryConstraint {
    /**
     * Gets the maximum velocity this constraint allows for a path state.
     *
     * @param state the path state.
     * @return the maximum velocity.
     */
    public double getMaxVelocity(Path.State state) {
        return Double.POSITIVE_INFINITY;
    }

    /**
     * Gets the maximum acceleration this constraint allows for a path state and velocity.
     *
     * @param state    the path state.
     * @param velocity the velocity.
     * @return the maximum acceleration.
     */
    public double getMaxAcceleration(Path.State state, double velocity) {
        return Double.POSITIVE_INFINITY;
    }

    /**
     * Gets the maximum deceleration this constraint allows for a path state and velocity.
     *
     * @param state    the path state.
     * @param velocity the velocity.
     * @return the maximum deceleration.s
     */
    public double getMaxDeceleration(Path.State state, double velocity) {
        return getMaxAcceleration(state, velocity);
    }
}
