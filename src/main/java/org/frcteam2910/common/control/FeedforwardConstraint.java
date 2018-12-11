package org.frcteam2910.common.control;

import org.frcteam2910.common.math.MathUtils;

/**
 * Constrain both the velocity and acceleration based on the feedforward equation <code>F = kV * V + kA * A + kS</code>.
 * <p>
 * See the <a href="https://www.chiefdelphi.com/media/papers/3402">drivetrain characterization paper</a> for information
 * on how to determine these values.
 */
public class FeedforwardConstraint implements ITrajectoryConstraint {
    private final double targetFeedforward;

    private final double kV;
    private final double kA;
    private final double kS;

    public FeedforwardConstraint(double targetFeedforward, double kV, double kA, double kS) {
        if (targetFeedforward < 0.0) {
            throw new IllegalArgumentException("Target feedforward must be positive");
        }
        if (kV < 0.0) {
            throw new IllegalArgumentException("kV must be positive");
        }
        if (kA < 0.0) {
            throw new IllegalArgumentException("kA must be positive");
        }
        if (kS < 0.0) {
            throw new IllegalArgumentException("kS must be positive");
        }



        this.targetFeedforward = targetFeedforward;
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
    }

    @Override
    public double getMaxVelocity(PathSegment segment) {
        return (targetFeedforward - kS) / kV;
    }

    @Override
    public double getMaxAcceleration(PathSegment segment, double velocity) {
        double accel = (targetFeedforward - kV * velocity - kS) / kA;
        if (MathUtils.epsilonEquals(accel, 0.0)) {
            return 0.0;
        }

        return accel;
    }
}
