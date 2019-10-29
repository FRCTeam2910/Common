package org.frcteam2910.common.control2;

public interface ITrajectoryConstraint {
    double getMaxVelocity(Path.State state);

    double getMaxAcceleration(Path.State state, double velocity);

    double getMaxDeceleration(Path.State state, double velocity);
}
