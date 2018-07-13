package org.frcteam2910.common.control;

import org.frcteam2910.common.Constants;
import org.frcteam2910.common.Logger;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.MovingAverage;
import org.junit.Test;

public class TrajectoryTest {
	public static final Trajectory.Constraints CONSTRAINTS = new Trajectory.Constraints();

	public static final int SPEED_RUNS = 10;
	public static final double SPEED_DT = 20 / Constants.MILLISECONDS;

	static {
		CONSTRAINTS.maxVelocity = 12;
		CONSTRAINTS.maxAcceleration = 5.5;
		CONSTRAINTS.arcVelocityScalar = 1. / 30.;
	}

	@Test
	public void speedTest() {
		Logger logger = new Logger("TrajectoryTest.speedTest");

		Path path = PathBuilder.build(
				new Waypoint(new Vector2(0, 0), 0),
				new Waypoint(new Vector2(5, 100), 30),
				new Waypoint(new Vector2(-100, 80), 10),
				new Waypoint(new Vector2(-80, 110), 20),
				new Waypoint(new Vector2(-50, 200), 0)
		);
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
