package org.frcteam2910.common.control;

import org.frcteam2910.common.Constants;
import org.frcteam2910.common.Logger;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.MovingAverage;
import org.junit.Ignore;
import org.junit.Test;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;

import static org.hamcrest.Matchers.lessThanOrEqualTo;
import static org.junit.Assert.assertThat;

public class TrajectoryTest {
	public static final Trajectory.Constraints CONSTRAINTS = new Trajectory.Constraints();

	public static final int SPEED_RUNS = 10;
	public static final double SPEED_DT = 20 / Constants.MILLISECONDS;

	private static final Waypoint[] WAYPOINTS = {
			new Waypoint(new Vector2(0, 0), Rotation2.fromDegrees(90), Rotation2.fromDegrees(90)),
			new Waypoint(new Vector2(0, 50), Rotation2.fromDegrees(90), Rotation2.fromDegrees(0))
	};

	static {
		CONSTRAINTS.maxVelocity = 12;
		CONSTRAINTS.maxAcceleration = 5.5;
		CONSTRAINTS.arcVelocityScalar = 1.0 / 30.0;
	}

	@Test
	@Ignore
	public void velocityContinuityTest() {
		Path path = new SplinePathGenerator().generate(WAYPOINTS);
		Trajectory trajectory = new Trajectory(path, CONSTRAINTS);
		trajectory.calculateSegments(SPEED_DT);

		final double maxAllowableAbsDeltaVelocity = CONSTRAINTS.maxAcceleration * SPEED_DT * (1 + MathUtils.EPSILON);

		Trajectory.Segment previousSegment = null;
		for (Trajectory.Segment segment : trajectory.getSegments()) {
			if (previousSegment == null) {
				previousSegment = segment;
				continue;
			}

			double absDeltaVelocity = Math.abs(previousSegment.velocity - segment.velocity);

			assertThat("Actual acceleration exceeds max acceleration", absDeltaVelocity, lessThanOrEqualTo(maxAllowableAbsDeltaVelocity));

			previousSegment = segment;
		}
	}

	@Test
	public void speedTest() {
		Logger logger = new Logger("TrajectoryTest.speedTest");

		Path loggedPath = new SplinePathGenerator().generate(WAYPOINTS);
		Trajectory loggedTrajectory = new Trajectory(loggedPath, CONSTRAINTS);

		MovingAverage average = new MovingAverage(SPEED_RUNS);

		double highTime = Double.NEGATIVE_INFINITY, lowTime = Double.POSITIVE_INFINITY;

		for (int run = 1; run <= SPEED_RUNS; run++) {
			long start = System.nanoTime();

			Path path = new SplinePathGenerator().generate(WAYPOINTS);
			Trajectory trajectory = new Trajectory(path, CONSTRAINTS);
			trajectory.calculateSegments(SPEED_DT);

			long end = System.nanoTime();

			double time = (end - start) * (Constants.MILLISECONDS / Constants.NANOSECONDS);

			if (time > highTime)
				highTime = time;
			if (time < lowTime)
				lowTime = time;

			average.add(time);
		}

		logger.info("Generated %d trajectories", SPEED_RUNS);
		logger.info("Trajectory duration: %.3f s", loggedTrajectory.getDuration());
		logger.info("Segments calculated for every %.3f ms", SPEED_DT * Constants.MILLISECONDS);
		logger.info("Longest generation time: %.3f ms", highTime);
		logger.info("Shortest generation time: %.3f ms", lowTime);
		logger.info("Average trajectory time: %.3f ms", average.get());
	}

	@Test
	@Ignore
	public void writeCsv() {
		Path path = new SplinePathGenerator().generate(WAYPOINTS);
		Trajectory trajectory = new Trajectory(path, CONSTRAINTS);
		trajectory.calculateSegments(SPEED_DT);

		try (PrintStream out = new PrintStream(new FileOutputStream("trajectory.csv"))) {
			out.printf("time,x,y,heading,rotation,position,velocity,acceleration%n");
			for (Trajectory.Segment segment : trajectory.getSegments()) {
				out.printf("%f,%f,%f,%f,%f,%f,%f,%f%n", segment.time, segment.translation.x, segment.translation.y,
						segment.heading.toDegrees(), segment.rotation.toDegrees(), segment.position,
						segment.velocity, segment.acceleration);
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
