package org.frcteam2910.common.control;

import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;
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

	public Trajectory(Path path, Constraints constraints) {
		this.path = path;
		this.profiles = new MotionProfile[path.getSegments().size()];

		MotionProfile.Goal lastPosition = new MotionProfile.Goal(0, 0);
		for (int i = 0; i < profiles.length; i++) {
			Path.Segment pathSegment = path.getSegments().get(i);

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
	}

	private static double getMaxVelocityForSegment(Path.Segment pathSegment, Constraints constraints) {
		// We can't always follow an arc at max velocity. Scale down the max velocity so we can follow the arc
		if (pathSegment instanceof Path.Segment.Arc) {
			Path.Segment.Arc arc = (Path.Segment.Arc) pathSegment;
			return Math.min(constraints.maxVelocity, constraints.maxVelocity * arc.getRadius() * constraints.arcVelocityScalar);
		}
		return constraints.maxVelocity;
	}

	private MotionProfile.State calculateState(double time) {
		int profileIndex = 0;
		while (profileIndex < profiles.length - 1 && time > profiles[profileIndex].getDuration()) {
			time -= profiles[profileIndex++].getDuration();
		}

		return profiles[profileIndex].calculate(time);
	}

	public Segment calculateSegment(double time, double dt) {
		MotionProfile.State state = calculateState(time);
		Translation2d pathPosition = path.getPositionAtDistance(state.position);
		Rotation2d pathHeading = path.getSlopeAtDistance(state.position);

		return new Segment(dt, pathPosition, pathHeading, state.position, state.velocity, state.acceleration);
	}

	public void calculateSegments(double dt) {
		int segmentCount = (int) (getDuration() / dt);
		segments = new Segment[segmentCount];

		for (int i = 0; i < segmentCount; i++) {
			segments[i] = calculateSegment(i * dt, dt);
		}
	}

	public Segment[] getSegments() {
		return segments;
	}

	public double getDuration() {
		double time = 0;
		for (MotionProfile profile : profiles)
			time += profile.getDuration();
		return time;
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
		public final double dt;
		public final Translation2d translation;
		public final Rotation2d heading;
		public final double position, velocity, acceleration;

		public Segment(double dt, Translation2d translation, Rotation2d heading, double position, double velocity,
		               double acceleration) {
			this.dt = dt;
			this.translation = translation;
			this.heading = heading;
			this.position = position;
			this.velocity = velocity;
			this.acceleration = acceleration;
		}
	}
}
