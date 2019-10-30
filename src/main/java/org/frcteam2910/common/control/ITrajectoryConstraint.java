package org.frcteam2910.common.control;

public interface ITrajectoryConstraint {
    /**
     * Gets the maximum velocity this constraint allows for a path state.
     *
     * @param state the path state.
     * @return the maximum velocity.
     */
    double getMaxVelocity(Path.State state);

    /**
     * Gets the maximum acceleration this constraint allows for a path state and velocity.
     *
     * @param state    the path state.
     * @param velocity the velocity.
     * @return the maximum acceleration.
     */
    double getMaxAcceleration(Path.State state, double velocity);

    /**
     * Gets the maximum deceleration this constraint allows for a path state and velocity.
     *
     * @param state    the path state.
     * @param velocity the velocity.
     * @return the maximum deceleration.s
     */
    double getMaxDeceleration(Path.State state, double velocity);
}
