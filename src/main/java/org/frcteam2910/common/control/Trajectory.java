package org.frcteam2910.common.control;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.motion.MotionProfile;
import org.frcteam2910.common.motion.TrapezoidalMotionProfile;

/**
 * A trajectory describes how a path is followed.
 * It contains all the motion profiles for a path.
 */
public class Trajectory {
	private final Path path;
	private final MotionProfile[] profiles;
	private Segment[] segments;
	private double duration = 0.0;

	private double[] maxSegmentVelocities;

	public Trajectory(Path path, Constraints constraints) {
		this.path = path;

		maxSegmentVelocities = new double[path.getSegments().size()];

		// First iterate forwards, finding the max velocities when we accelerate
		for (int i = 0; i < maxSegmentVelocities.length; i++) {
			PathSegment pathSegment = path.getSegments().get(i);

			double maxVelocity = constraints.maxVelocity;
			// Max velocity due to max centripetal acceleration
			maxVelocity = Math.min(maxVelocity,
                    Math.sqrt(constraints.maxCentripetalAcceleration / pathSegment.getCurvature()));

			double startVelocity = 0.0;
			if (i != 0) {
				startVelocity = maxSegmentVelocities[i - 1];
			}

			// How much do we have to change our velocity by?
			double deltaVelocity = maxVelocity - startVelocity;
            if (deltaVelocity < 0) {
                maxSegmentVelocities[i] = maxVelocity;
                continue;
            }

			// How long does it take to increase the velocity by deltaVelocity?
			double accelTime = deltaVelocity / constraints.maxAcceleration;

			// How far do we move while accelerating to the max velocity?
			double accelDist = 0.5 * constraints.maxAcceleration * accelTime * accelTime + startVelocity * accelTime;

			if (accelDist > pathSegment.getLength()) {
			    // We can't reach the segment's max velocity.
                // What is the max velocity we can achieve?
                double[] roots = MathUtils.quadratic(0.5 * constraints.maxAcceleration, startVelocity, -pathSegment.getLength());

                double maxAllowableAccelTime = Math.max(roots[0], roots[1]);

                maxVelocity = startVelocity + constraints.maxAcceleration * maxAllowableAccelTime;
            }

            maxSegmentVelocities[i] = maxVelocity;
		}

        // Now iterate backwards, finding the max velocities when we decelerate
        for (int i = maxSegmentVelocities.length - 1; i >= 0; i--) {
            PathSegment pathSegment = path.getSegments().get(i);

            double maxVelocity = maxSegmentVelocities[i];

            double endVelocity = 0.0;
            if (i != maxSegmentVelocities.length - 1) {
                endVelocity = maxSegmentVelocities[i + 1];
            }

            // How much do we have to change our velocity by?
            double deltaVelocity = endVelocity - maxVelocity;
            if (deltaVelocity > 0) {
                continue;
            }

            // How long does it take to decrease the velocity by deltaVelocity?
            double decelTime = Math.abs(deltaVelocity) / constraints.maxAcceleration;

            // How far do we move while decelerating to the end velocity?
            double decelDist = 0.5 * constraints.maxAcceleration * decelTime * decelTime + endVelocity * decelTime;

            if (decelDist > pathSegment.getLength()) {
                // We can't decelerate fast enough.
                // What is the max velocity we can decelerate from?
                double[] roots = MathUtils.quadratic(0.5 * constraints.maxAcceleration, endVelocity, -pathSegment.getLength());

                double maxAllowableDecelTime = Math.max(roots[0], roots[1]);

                maxVelocity = endVelocity + constraints.maxAcceleration * maxAllowableDecelTime;
            }

            maxSegmentVelocities[i] = maxVelocity;
        }

		this.profiles = new MotionProfile[path.getSegments().size()];
		MotionProfile.Goal lastPosition = new MotionProfile.Goal(0, 0);
		for (int i = 0; i < profiles.length; i++) {
			PathSegment pathSegment = path.getSegments().get(i);

			// Create the motion constraints for this segment
			MotionProfile.Constraints segmentConstraints = new MotionProfile.Constraints(maxSegmentVelocities[i], constraints.maxAcceleration);

			// Look ahead to see the end velocity for the segment
			double endVelocity = 0;
			if (i < profiles.length - 1) {
				endVelocity = maxSegmentVelocities[i + 1];
			}

			MotionProfile.Goal endPosition = new MotionProfile.Goal(lastPosition.position + pathSegment.getLength(), endVelocity);
			profiles[i] = new TrapezoidalMotionProfile(lastPosition, endPosition, segmentConstraints);

			// The profile may not have been able to finish accelerating. We need to manually find the ending velocity
			lastPosition = new MotionProfile.Goal(profiles[i].calculate(profiles[i].getDuration()));
		}

		for (MotionProfile profile : profiles) {
			duration += profile.getDuration();
		}
	}

	public Segment calculateSegment(double time) {
		int profileIndex = 0;
		double profileTime = time;
		while (profileIndex < profiles.length - 1 && profileTime > profiles[profileIndex].getDuration()) {
			profileTime -= profiles[profileIndex++].getDuration();
		}

		MotionProfile profile = profiles[profileIndex];
		PathSegment segment = path.getSegments().get(profileIndex);

		MotionProfile.State state = profiles[profileIndex].calculate(profileTime);
		Vector2 pathPosition = path.getPositionAtDistance(state.position);
		Rotation2 pathHeading = path.getHeadingAtDistance(state.position);
//		Rotation2 pathRotation = segment.getRotationAtPercentage(profileTime / profile.getDuration());
		Rotation2 pathRotation = path.getRotationAtDistance(path.getLength() * (time / getDuration()));

		return new Segment(profileIndex, time, pathPosition, pathHeading, pathRotation, state.position, state.velocity,
                state.acceleration, maxSegmentVelocities[profileIndex]);
	}

	public void calculateSegments(double dt) {
		int segmentCount = (int) (getDuration() / dt);
		segments = new Segment[segmentCount];

		for (int i = 0; i < segmentCount; i++) {
			segments[i] = calculateSegment(i * dt);
		}
	}

	public Segment[] getSegments() {
		return segments;
	}

	public double getDuration() {
		return duration;
	}

	public static class Constraints {
		/**
		 * The maximum velocity of the robot
		 */
		public double maxVelocity;

		/**
		 * The maximum acceleration of the robot
		 */
		public double maxAcceleration;

		/**
         * The maximum centripetal acceleration of the robot. This is responsible for slowing the robot down as it
         * approaches a curve in the path.
         *
		 * Decrease this if arcs are not being followed correctly and increase it if it is too slow
		 * when following arcs.
		 */
		public double maxCentripetalAcceleration;
	}

	public static class Segment {
	    public final int pathSegmentIndex;
		public final double time;
		public final Vector2 translation;
		public final Rotation2 heading;
		public final Rotation2 rotation;
		public final double position, velocity, acceleration;
		public final double maxVelocity;

		private Segment(int pathSegmentIndex, double time, Vector2 translation, Rotation2 heading, Rotation2 rotation,
                       double position, double velocity, double acceleration, double maxVelocity) {
		    this.pathSegmentIndex = pathSegmentIndex;
			this.time = time;
			this.translation = translation;
			this.heading = heading;
			this.rotation = rotation;
			this.position = position;
			this.velocity = velocity;
			this.acceleration = acceleration;
			this.maxVelocity = maxVelocity;
		}
	}
}
