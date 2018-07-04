package org.frcteam2910.common.control;

import com.team254.lib.util.MovingAverage;
import com.team254.lib.util.math.Translation2d;
import org.frcteam2910.common.Constants;
import org.frcteam2910.common.Logger;
import org.junit.Test;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

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
				new Waypoint(new Translation2d(0, 0), 0),
				new Waypoint(new Translation2d(5, 100), 30),
				new Waypoint(new Translation2d(-100, 80), 10),
				new Waypoint(new Translation2d(-80, 110), 20),
				new Waypoint(new Translation2d(-50, 200), 0)
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

			average.addNumber(time);
		}

		logger.info("Generated %d trajectories", SPEED_RUNS);
		logger.info("Trajectory duration: %.3f s", loggedTrajectory.getDuration());
		logger.info("Segments calculated for every %.3f ms", SPEED_DT * Constants.MILLISECONDS);
		logger.info("Longest generation time: %.3f ms", highTime);
		logger.info("Shortest generation time: %.3f ms", lowTime);
		logger.info("Average trajectory time: %.3f ms", average.getAverage());
	}
}
