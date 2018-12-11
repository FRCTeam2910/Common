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
	private double[] maxSegmentAccelerations;

	public Trajectory(Path path, ITrajectoryConstraint... constraints) {
		this.path = path;

		maxSegmentVelocities = new double[path.getSegments().size()];
		maxSegmentAccelerations = new double[path.getSegments().size()];

		// First iterate forwards, finding the max velocities when we accelerate
		for (int i = 0; i < maxSegmentVelocities.length; i++) {
			PathSegment pathSegment = path.getSegments().get(i);

			double startVelocity = 0.0;
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

			double endVelocity = 0.0;
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
				throw new RuntimeException("Illegal max acceleration");
			}

			// Now check if we can reach our max velocity
			double deltaVelocity = maxVelocity - endVelocity;
			if (deltaVelocity > 0.0) {
				double accelTime = deltaVelocity / maxAcceleration;

				double accelDist = 0.5 * maxAcceleration * accelTime * accelTime + endVelocity * accelTime;

				if (accelDist > pathSegment.getLength()) {
					double[] roots = MathUtils.quadratic(0.5 * maxAcceleration, endVelocity, -pathSegment.getLength());

					double maxAllowableAccelTime = Math.max(roots[0], roots[1]);

					maxVelocity = endVelocity + maxAcceleration * maxAllowableAccelTime;
				}
			}

			maxSegmentVelocities[i] = Math.min(maxSegmentVelocities[i], maxVelocity);
			maxSegmentAccelerations[i] = Math.max(maxSegmentAccelerations[i], maxAcceleration);
		}

		this.profiles = new MotionProfile[path.getSegments().size()];
		MotionProfile.Goal lastPosition = new MotionProfile.Goal(0, 0);
		for (int i = 0; i < profiles.length; i++) {
			PathSegment pathSegment = path.getSegments().get(i);

			// Create the motion constraints for this segment
			MotionProfile.Constraints segmentConstraints = new MotionProfile.Constraints(maxSegmentVelocities[i], Math.max(maxSegmentAccelerations[i], MathUtils.EPSILON));

			// Look ahead to see the end velocity for the segment
			double endVelocity = 0;
			if (i < profiles.length - 1) {
				endVelocity = maxSegmentVelocities[i + 1];
			}

			MotionProfile.Goal endPosition = new MotionProfile.Goal(lastPosition.position + pathSegment.getLength(), endVelocity);
			profiles[i] = new TrapezoidalMotionProfile(lastPosition, endPosition, segmentConstraints);

			duration += profiles[i].getDuration();

			// The profile may not have been able to finish accelerating. We need to manually find the ending velocity
			lastPosition = new MotionProfile.Goal(profiles[i].calculate(profiles[i].getDuration()));
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

	public static class Segment {
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
