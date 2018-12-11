package org.frcteam2910.common.control;

import org.frcteam2910.common.math.MathUtils;

/**
 * Motion constraints for the robot.
 *
 * See the drivetrain characterization paper for information on how to determine these values
 * <a href="https://www.chiefdelphi.com/media/papers/3402">here</a>.
 */
public class FeedforwardConstraint implements ITrajectoryConstraint {
    private final double targetFeedforward;

    private final double kV;
    private final double kA;
    private final double kS;

    public FeedforwardConstraint(double targetFeedforward, double kV, double kA, double kS) {
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
