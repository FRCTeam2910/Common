package org.frcteam2910.common.control;

import org.frcteam2910.common.math.MathUtils;

import java.util.ArrayList;
import java.util.List;

public class Trajectory {
    private final Path path;

    private final double duration;

    private List<ConstrainedPathState> constrainedPathStates = new ArrayList<>();
    private double[] pathStateStartTimes;

    public Trajectory(Path path, TrajectoryConstraint[] trajectoryConstraints, double sampleDistance) {
        this.path = path;

        double distance = 0.0;
        ConstrainedPathState lastState = new ConstrainedPathState(
                path.calculate(distance),
                0.0,
                0.0,
                0.0, // Trajectory starting velocity
                0.0
        );
        while (distance < path.getLength()) {
            Path.State startingState = path.calculate(distance);

            double profileLength = sampleDistance;
            if (distance + sampleDistance > path.getLength()) {
                profileLength = path.getLength() - distance;
            }

            Path.State endingState = path.calculate(distance + profileLength);

            double startingVelocity = lastState.endingVelocity;

            double maxEndingVelocity = Double.POSITIVE_INFINITY;

            for (TrajectoryConstraint constraint : trajectoryConstraints) {
                maxEndingVelocity = Math.min(constraint.getMaxVelocity(endingState), maxEndingVelocity);
            }

            ConstrainedPathState state = new ConstrainedPathState(
                    startingState,
                    profileLength,
                    startingVelocity,
                    maxEndingVelocity,
                    0.0
            );

            // If the max ending velocity is lower than the starting velocity we know that we have to decelerate
            double maxDeltaVelocity = maxEndingVelocity - startingVelocity;

            // Calculate the optimal acceleration for this profile
            double optimalAcceleration = Math.pow(maxDeltaVelocity, 2.0) / (2.0 * profileLength) + (startingVelocity / profileLength) * maxDeltaVelocity;
            if (MathUtils.epsilonEquals(optimalAcceleration, 0.0)) {
                // We are neither accelerating or decelerating
                state.acceleration = 0.0;
                state.endingVelocity = state.startingVelocity;
            } else if (optimalAcceleration > 0.0) {
                // We are accelerating
                double maxStartingAcceleration = Double.POSITIVE_INFINITY;
                double maxEndingAcceleration = Double.POSITIVE_INFINITY;
                for (TrajectoryConstraint constraint : trajectoryConstraints) {
                    maxStartingAcceleration = Math.min(constraint.getMaxAcceleration(startingState, startingVelocity), maxStartingAcceleration);
                    maxEndingAcceleration = Math.min(constraint. getMaxAcceleration(endingState, startingVelocity), maxEndingAcceleration); // TODO: Use endingVelocity instead of startingVelocity
                }

                // Take the lower of the two accelerations
                double acceleration = Math.min(maxStartingAcceleration, maxEndingAcceleration);

                // Use the optimal acceleration if we can
                acceleration = Math.min(acceleration, optimalAcceleration);

                // Find the maximum velocity we can reach during this profile
                double[] roots = MathUtils.quadratic(0.5 * acceleration, startingVelocity, -profileLength);
                double duration = Math.max(roots[0], roots[1]);

                state.endingVelocity = startingVelocity + acceleration * duration;
                state.acceleration = acceleration;
            } else {
                // If we can decelerate before we reach the end of the profile, use that deceleration.
                // This acceleration may not be achievable. When we go over the trajectory in reverse we will take care
                // of this.
                state.acceleration = optimalAcceleration;
            }

            constrainedPathStates.add(state);
            lastState = state;

            distance += profileLength;
        }

        for (int i = constrainedPathStates.size() - 1; i >= 0; i--) {
            ConstrainedPathState constrainedState = constrainedPathStates.get(i);

            constrainedState.endingVelocity = 0.0; // Trajectory ending velocity
            if (i != constrainedPathStates.size() - 1) {
                constrainedState.endingVelocity = constrainedPathStates.get(i + 1).startingVelocity;
            }

            // Check if we are decelerating
            double deltaVelocity = constrainedState.endingVelocity - constrainedState.startingVelocity;
            if (deltaVelocity < 0.0) {
                // Use the deceleration constraint for when we decelerate
                double deceleration = Double.POSITIVE_INFINITY;
                for (TrajectoryConstraint constraint : trajectoryConstraints) {
                    deceleration = Math.min(deceleration, constraint.getMaxDeceleration(constrainedState.pathState, constrainedState.endingVelocity));
                }

                // Find how long it takes for us to decelerate to the ending velocity
                double decelTime = deltaVelocity / -deceleration;

                // Find how far we travel while decelerating
                double decelDist = 0.5 * deceleration * Math.pow(decelTime, 2.0) + constrainedState.endingVelocity * decelTime;

                // If we travel too far we have to decrease the starting velocity
                if (decelDist > constrainedState.length) {
                    // We can't decelerate in time. Change the starting velocity of the segment so we can.
                    double[] roots = MathUtils.quadratic(0.5 * deceleration, constrainedState.endingVelocity, -constrainedState.length);

                    // Calculate the maximum time that we can decelerate
                    double maxAllowableDecelTime = Math.max(roots[0], roots[1]);

                    // Find what are starting velocity can be in order to end at our ending velocity
                    constrainedState.acceleration = -deceleration;
                    constrainedState.startingVelocity = constrainedState.endingVelocity + deceleration * maxAllowableDecelTime;
                }
            }
        }

        pathStateStartTimes = new double[constrainedPathStates.size()];

        double duration = 0.0;
        for (int i = 0; i < constrainedPathStates.size(); i++) {
            pathStateStartTimes[i] = duration;
            duration += constrainedPathStates.get(i).getDuration();
        }
        this.duration = duration;
    }

    public State calculate(double time) {
        int start = 0;
        int end = constrainedPathStates.size() - 1;
        int mid = start + (end - start) / 2;
        while (start <= end) {
            mid = (start + end) / 2;

            if (time > pathStateStartTimes[mid] + constrainedPathStates.get(mid).getDuration()) {
                start = mid + 1;
            } else if (time < pathStateStartTimes[mid]) {
                end = mid - 1;
            } else {
                break;
            }
        }

        ConstrainedPathState constrainedPathState = constrainedPathStates.get(mid);
        return constrainedPathState.calculate(time - pathStateStartTimes[mid]);
    }

    public double getDuration() {
        return duration;
    }

    public Path getPath() {
        return path;
    }

    class ConstrainedPathState {
        public Path.State pathState;
        public double length;
        public double startingVelocity;
        public double endingVelocity;
        public double acceleration;

        public ConstrainedPathState(Path.State pathState, double length, double startingVelocity, double endingVelocity, double acceleration) {
            this.pathState = pathState;
            this.length = length;
            this.startingVelocity = startingVelocity;
            this.endingVelocity = endingVelocity;
            this.acceleration = acceleration;
        }

        public double getDuration() {
            if (MathUtils.epsilonEquals(acceleration, 0.0)) {
                return length / startingVelocity;
            }

            if (MathUtils.epsilonEquals(endingVelocity, 0.0)) {
                return (startingVelocity / -acceleration);
            }

            double[] roots = MathUtils.quadratic(0.5 * acceleration, startingVelocity, -length);

            if (acceleration > 0.0) {
                return Math.max(roots[0], roots[1]);
            } else {
                return Math.min(roots[0], roots[1]);
            }
        }

        public State calculate(double time) {
            time = MathUtils.clamp(time, 0.0, getDuration());

            double distance = 0.5 * acceleration * Math.pow(time, 2.0) + startingVelocity * time + pathState.getDistance();

            return new State(
                    path.calculate(distance),
                    acceleration * time + startingVelocity,
                    acceleration
            );
        }
    }

    public static class State {
        private final Path.State pathState;
        private final double velocity;
        private final double acceleration;

        public State(Path.State pathState, double velocity, double acceleration) {
            this.pathState = pathState;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }

        public Path.State getPathState() {
            return pathState;
        }

        public double getVelocity() {
            return velocity;
        }

        public double getAcceleration() {
            return acceleration;
        }
    }
}
