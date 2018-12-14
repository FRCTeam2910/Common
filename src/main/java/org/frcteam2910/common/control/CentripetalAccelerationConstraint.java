package org.frcteam2910.common.control;

/**
 * A constraint that limits velocity by not allowing the centripetal acceleration to exceed a specified amount.
 * <p>
 * Limiting centripetal acceleration will slow down the robot as it tries to take a turn. The higher it is the less it
 * will slow down.
 * <p>
 * For more information on centripetal acceleration see <a href="https://www.youtube.com/watch?v=NH1_sO8QY3o">this Khan Academy video</a>.
 */
public final class CentripetalAccelerationConstraint implements ITrajectoryConstraint {
    private final double maxCentripetalAcceleration;

    /**
     * @param maxCentripetalAcceleration the maximum centripetal acceleration
     */
    public CentripetalAccelerationConstraint(double maxCentripetalAcceleration) {
        if (maxCentripetalAcceleration < 0.0) {
            throw new IllegalArgumentException("Centripetal acceleration must be positive");
        }

        this.maxCentripetalAcceleration = maxCentripetalAcceleration;
    }

    @Override
    public double getMaxVelocity(PathSegment segment) {
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
        if (segment.getCurvature() == 0.0) {
            return Double.POSITIVE_INFINITY;
        }

        return Math.sqrt(Math.abs(maxCentripetalAcceleration / segment.getCurvature()));
    }

    @Override
    public double getMaxAcceleration(PathSegment segment, double velocity) {
        return Double.POSITIVE_INFINITY; // We don't constrain acceleration
    }
}
