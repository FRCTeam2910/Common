package org.frcteam2910.common.control;

import org.frcteam2910.common.Constants;
import org.frcteam2910.common.Logger;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.math.spline.CubicHermiteSpline;
import org.frcteam2910.common.util.MovingAverage;
import org.junit.Ignore;
import org.junit.Test;

import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;

public class TrajectoryTest {
	public static final Trajectory.Constraints CONSTRAINTS = new Trajectory.Constraints();

	public static final int SPEED_RUNS = 10;
	public static final double SPEED_DT = 20 / Constants.MILLISECONDS;

	private static final Waypoint[] WAYPOINTS = {
			new Waypoint(new Vector2(0, 0), Rotation2.fromDegrees(90)),
			new Waypoint(new Vector2(50, 50), Rotation2.fromDegrees(90))
	};

	static {
		CONSTRAINTS.maxVelocity = 12;
		CONSTRAINTS.maxAcceleration = 5.5;
		CONSTRAINTS.arcVelocityScalar = 1.0 / 30.0;
	}

	@Test
	@Ignore("Iterating through the path to find segments takes forever. Test takes > 10 seconds")
	public void speedTest() {
		Logger logger = new Logger("TrajectoryTest.speedTest");

		Path path = new SplinePathGenerator().generate(WAYPOINTS);
		Trajectory loggedTrajectory = new Trajectory(path, CONSTRAINTS);

		MovingAverage average = new MovingAverage(SPEED_RUNS);

		double highTime = Double.NEGATIVE_INFINITY, lowTime = Double.POSITIVE_INFINITY;

		for (int run = 1; run <= SPEED_RUNS; run++) {
			long start = System.nanoTime();

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
}
