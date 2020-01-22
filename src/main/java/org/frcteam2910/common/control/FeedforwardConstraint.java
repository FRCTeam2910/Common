package org.frcteam2910.common.control;

/**
 * Constrain both the velocity and acceleration based on the equation <code>F = K<sub>V</sub>*V + K<sub>A</sub>*A</code>.
 */
public class FeedforwardConstraint extends TrajectoryConstraint {
    private final double targetFeedforward;

    private final double kV;
    private final double kA;

    private final boolean fastDeceleration;

    public FeedforwardConstraint(double targetFeedforward, double kV, double kA) {
        this(targetFeedforward, kV, kA, true);
    }

    public FeedforwardConstraint(double targetFeedforward, double kV, double kA, boolean fastDeceleration) {
        this.targetFeedforward = targetFeedforward;
        this.kV = kV;
        this.kA = kA;
        this.fastDeceleration = fastDeceleration;
    }

    @Override
    public double getMaxVelocity(Path.State state) {
        return targetFeedforward / kV;
    }

    @Override
    public double getMaxAcceleration(Path.State state, double velocity) {
        return (targetFeedforward - kV * velocity) / kA;

    }

    @Override
    public double getMaxDeceleration(Path.State state, double velocity) {
        return getMaxAcceleration(
                state,
                fastDeceleration ? -velocity : velocity
        );
    }
}
