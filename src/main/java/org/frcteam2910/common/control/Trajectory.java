package org.frcteam2910.common.control;

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

	public Trajectory(Path path, Constraints constraints) {
		this.path = path;
		this.profiles = new MotionProfile[path.getSegments().size()];

		MotionProfile.Goal lastPosition = new MotionProfile.Goal(0, 0);
		for (int i = 0; i < profiles.length; i++) {
			PathSegment pathSegment = path.getSegments().get(i);

			// Create the motion constraints for this segment
			double maxSegmentVelocity = getMaxVelocityForSegment(pathSegment, constraints);
			MotionProfile.Constraints segmentConstraints = new MotionProfile.Constraints(maxSegmentVelocity, constraints.maxAcceleration);

			// Look ahead to see the end velocity for the segment
			double endVelocity;
			if (i == profiles.length - 1)
				endVelocity = 0;
			else
				endVelocity = getMaxVelocityForSegment(path.getSegments().get(i + 1), constraints);
			endVelocity = Math.min(maxSegmentVelocity, endVelocity);

			MotionProfile.Goal endPosition = new MotionProfile.Goal(lastPosition.position + pathSegment.getLength(), endVelocity);
			profiles[i] = new TrapezoidalMotionProfile(lastPosition, endPosition, segmentConstraints);

			// The profile may not have been able to finish accelerating. We need to manually find the ending velocity
			lastPosition = new MotionProfile.Goal(profiles[i].calculate(profiles[i].getDuration()));
		}

		for (MotionProfile profile : profiles) {
			duration += profile.getDuration();
		}
	}

	private static double getMaxVelocityForSegment(PathSegment pathSegment, Constraints constraints) {
		// We can't always follow an arc at max velocity. Scale down the max velocity so we can follow the arc
		if (pathSegment instanceof PathArcSegment) {
			PathArcSegment arc = (PathArcSegment) pathSegment;
			return Math.min(constraints.maxVelocity, constraints.maxVelocity * arc.getRadius() * constraints.arcVelocityScalar);
		}
		return constraints.maxVelocity;
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

		return new Segment(time, pathPosition, pathHeading, pathRotation, state.position, state.velocity, state.acceleration);
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
		 * This factor is multiplied by the radius of an arc and the max velocity to determine the velocity the arc
		 * should be driven at.
		 *
		 * Pretty much decrease the number if arcs are not being followed correctly and increase it if it is too slow
		 * when following arcs.
		 */
		public double arcVelocityScalar;
	}

	public static class Segment {
		public final double time;
		public final Vector2 translation;
		public final Rotation2 heading;
		public final Rotation2 rotation;
		public final double position, velocity, acceleration;

		public Segment(double time, Vector2 translation, Rotation2 heading, Rotation2 rotation, double position,
					   double velocity, double acceleration) {
			this.time = time;
			this.translation = translation;
			this.heading = heading;
			this.rotation = rotation;
			this.position = position;
			this.velocity = velocity;
			this.acceleration = acceleration;
		}
	}
}
