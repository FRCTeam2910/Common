package org.frcteam2910.common.control;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.motion.MotionProfile;
import org.frcteam2910.common.motion.TrapezoidalMotionProfile;

import java.io.Serializable;

/**
 * A trajectory describes how a path is followed.
 * It contains all the motion profiles for a path.
 */
public class Trajectory implements Serializable {
	private static final long serialVersionUID = -7533657200117435410L;

	private final Path path;
	private final MotionProfile[] profiles;
	private Segment[] segments;
	private double duration = 0.0;

	private double[] maxSegmentVelocities;
	private double[] maxSegmentAccelerations;

	private double[] profileStartTimes;

	public Trajectory(Path path, ITrajectoryConstraint... constraints) {
		this(0.0, 0.0, path, constraints);
	}

	public Trajectory(double startingVelocity, double endingVelocity, Path path, ITrajectoryConstraint... constraints) {
		this.path = path;

		maxSegmentVelocities = new double[path.getSegments().size()];
		maxSegmentAccelerations = new double[path.getSegments().size()];

		// First iterate forwards, finding the max velocities when we accelerate
		for (int i = 0; i < maxSegmentVelocities.length; i++) {
			PathSegment pathSegment = path.getSegments().get(i);

			double startVelocity = startingVelocity;
			if (i > 0) {
				startVelocity = maxSegmentVelocities[i - 1];
			}

			double maxVelocity = Double.POSITIVE_INFINITY;
			for (ITrajectoryConstraint constraint : constraints) {
				maxVelocity = Math.min(maxVelocity, constraint.getMaxVelocity(pathSegment));
			}
			if (maxVelocity < 0.0 || !Double.isFinite(maxVelocity)) {
				throw new RuntimeException("Illegal max velocity");
			}

			double maxAcceleration = Double.POSITIVE_INFINITY;
			for (ITrajectoryConstraint constraint : constraints) {
				maxAcceleration = Math.min(maxAcceleration, constraint.getMaxAcceleration(pathSegment, startVelocity));
			}
			if (maxAcceleration < 0.0 || !Double.isFinite(maxAcceleration)) {
				throw new RuntimeException("Illegal max acceleration");
			}

			// Now check if we can reach our max velocity
			double deltaVelocity = maxVelocity - startVelocity;
			if (deltaVelocity > 0.0) {
				double accelTime = deltaVelocity / maxAcceleration;

				double accelDist = 0.5 * maxAcceleration * accelTime * accelTime + startVelocity * accelTime;

				if (accelDist > pathSegment.getLength()) {
					double[] roots = MathUtils.quadratic(0.5 * maxAcceleration, startVelocity, -pathSegment.getLength());

					double maxAllowableAccelTime = Math.max(roots[0], roots[1]);

					maxVelocity = startVelocity + maxAcceleration * maxAllowableAccelTime;
				}
			}

            maxSegmentVelocities[i] = maxVelocity;
			maxSegmentAccelerations[i] = maxAcceleration;
		}

		for (int i = maxSegmentVelocities.length - 1; i >= 0; i--) {
			PathSegment pathSegment = path.getSegments().get(i);

			double endVelocity = endingVelocity;
			if (i < maxSegmentVelocities.length - 1) {
				endVelocity = maxSegmentVelocities[i + 1];
			}

			double maxVelocity = Double.POSITIVE_INFINITY;
			for (ITrajectoryConstraint constraint : constraints) {
				maxVelocity = Math.min(maxVelocity, constraint.getMaxVelocity(pathSegment));
			}
			if (maxVelocity < 0.0 || !Double.isFinite(maxVelocity)) {
				throw new RuntimeException("Illegal max velocity");
			}

			double maxAcceleration = Double.POSITIVE_INFINITY;
			for (ITrajectoryConstraint constraint : constraints) {
				maxAcceleration = Math.min(maxAcceleration, constraint.getMaxAcceleration(pathSegment, endVelocity));
			}
			if (maxAcceleration < 0.0 || !Double.isFinite(maxAcceleration)) {
				throw new RuntimeException("Illegal max acceleration: " + maxAcceleration);
			}

			// Now check if we can reach our max velocity
			double deltaVelocity = maxVelocity - endVelocity;
			if (deltaVelocity > 0.0) {
				double decelTime = deltaVelocity / maxAcceleration;

				double decelDist = 0.5 * maxAcceleration * decelTime * decelTime + endVelocity * decelTime;

				if (decelDist > pathSegment.getLength()) {
					double[] roots = MathUtils.quadratic(0.5 * maxAcceleration, endVelocity, -pathSegment.getLength());

					double maxAllowableDecelTime = Math.max(roots[0], roots[1]);

					maxVelocity = endVelocity + maxAcceleration * maxAllowableDecelTime;
				}
			}

			maxSegmentVelocities[i] = Math.min(maxSegmentVelocities[i], maxVelocity);
			maxSegmentAccelerations[i] = Math.max(maxSegmentAccelerations[i], maxAcceleration);
		}

		this.profiles = new MotionProfile[path.getSegments().size()];
		this.profileStartTimes = new double[profiles.length];

		MotionProfile.Goal lastPosition = new MotionProfile.Goal(0, startingVelocity);
		for (int i = 0; i < profiles.length; i++) {
			// Create the motion constraints for this segment
			MotionProfile.Constraints segmentConstraints = new MotionProfile.Constraints(maxSegmentVelocities[i], Math.max(maxSegmentAccelerations[i], MathUtils.EPSILON));

			// Look ahead to see the end velocity for the segment
			double endVelocity = endingVelocity;
			if (i < profiles.length - 1) {
				endVelocity = maxSegmentVelocities[i + 1];
			}

			MotionProfile.Goal endPosition = new MotionProfile.Goal(path.getDistanceToSegmentEnd(i), endVelocity);
			profiles[i] = new TrapezoidalMotionProfile(lastPosition, endPosition, segmentConstraints);

			profileStartTimes[i] = duration;
			duration += profiles[i].getDuration();

			// The profile may not have been able to finish accelerating. We need to manually find the ending velocity
			lastPosition = new MotionProfile.Goal(
					path.getDistanceToSegmentEnd(i),
					profiles[i].calculate(profiles[i].getDuration()).velocity
			);
		}
	}

	public Segment calculateSegment(double time) {
		int profileIndex;
		double profileTime;

		// Binary search to find the correct motion profile
		{
			int start = 0;
			int end = profiles.length - 1;
			int mid = start + (end - start) / 2;

			while (start <= end) {
				// Mid is halfway between start and end
				mid = (start + end) / 2;

				if (time > profileStartTimes[mid] + profiles[mid].getDuration()) {
					// Our time is greater than the end time of the profile, move start to mid and try again
					start = mid + 1;
				} else if (time < profileStartTimes[mid]) {
					// Our time is less than the start time of the profile, move end to mid and try again
					end = mid - 1;
				} else {
					// We are within the start and end times of our profile. This is the profile we want.
					break;
				}
			}

			profileIndex = mid;
			if (profileIndex >= profileStartTimes.length) {
				return new Segment(profileIndex, time, Vector2.ZERO, Rotation2.ZERO, Rotation2.ZERO, 0.0,
						0.0, 0.0, 0.0, 0.0);
			}

			profileTime = time - profileStartTimes[profileIndex];
		}


		MotionProfile.State state = profiles[profileIndex].calculate(profileTime);
		Vector2 pathPosition = path.getPositionAtDistance(state.position);
		Rotation2 pathHeading = path.getHeadingAtDistance(state.position);
		Rotation2 pathRotation = path.getRotationAtDistance(path.getLength() * (time / getDuration()));

		return new Segment(profileIndex, time, pathPosition, pathHeading, pathRotation, state.position, state.velocity,
                state.acceleration, maxSegmentVelocities[profileIndex], maxSegmentAccelerations[profileIndex]);
	}

	public void calculateSegments(double dt) {
		int segmentCount = (int) Math.ceil(getDuration() / dt);
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

	public static class Segment implements Serializable {
		private static final long serialVersionUID = 1777234890765500009L;

	    public final int pathSegmentIndex;
		public final double time;
		public final Vector2 translation;
		public final Rotation2 heading;
		public final Rotation2 rotation;
		public final double position, velocity, acceleration;
		public final double maxVelocity;
		public final double maxAcceleration;

		private Segment(int pathSegmentIndex, double time, Vector2 translation, Rotation2 heading, Rotation2 rotation,
                        double position, double velocity, double acceleration, double maxVelocity,
                        double maxAcceleration) {
		    this.pathSegmentIndex = pathSegmentIndex;
			this.time = time;
			this.translation = translation;
			this.heading = heading;
			this.rotation = rotation;
			this.position = position;
			this.velocity = velocity;
			this.acceleration = acceleration;
			this.maxVelocity = maxVelocity;
			this.maxAcceleration = maxAcceleration;
		}
	}
}
