package org.frcteam2910.common.control;

public class CentripetalAccelerationConstraint implements ITrajectoryConstraint {
    private final double maxCentripetalAcceleration;

    public CentripetalAccelerationConstraint(double maxCentripetalAcceleration) {
        this.maxCentripetalAcceleration = maxCentripetalAcceleration;
    }

    @Override
    public double getMaxVelocity(Path.State state) {
        // let A be the centripetal acceleration
        // let V be the max velocity
        // let C be the curvature of the path
        //
        // A = CV^2
        // A / C = V^2
        // sqrt(A / C) = V
        //
        // Curvature and max acceleration is always positive and we only expect a positive result so plus-minus is not
        // needed.

        // Special case when following a line, centripetal acceleration is 0 so don't constrain velocity
        if (state.getCurvature() == 0.0) {
            return Double.POSITIVE_INFINITY;
        }

        return Math.sqrt(Math.abs(maxCentripetalAcceleration / state.getCurvature()));
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
