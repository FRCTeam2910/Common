package org.frcteam2910.common.control2;

import com.github.sh0nk.matplotlib4j.Plot;
import com.github.sh0nk.matplotlib4j.PythonExecutionException;
import org.frcteam2910.common.Constants;
import org.frcteam2910.common.Logger;
import org.frcteam2910.common.control.Waypoint;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.MovingAverage;
import org.junit.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class TrajectoryTest {
    private static final double SAMPLE_DISTANCE = 1.0e-2;

    private static final double DT = 5.0e-3;

    private static final double KV = 0.8;
    private static final double KA = 0.1;
    private static final double MAX_FEEDFORWARD = 10.0;

    private static final Waypoint[] WAYPOINTS = {
            new Waypoint(new Vector2(0, 0), Rotation2.fromDegrees(90), Rotation2.fromDegrees(90)),
            new Waypoint(new Vector2(50.0, 50.0), Rotation2.fromDegrees(90), Rotation2.fromDegrees(0))
    };
    private static final ITrajectoryConstraint[] CONSTRAINTS = {
            new CentripetalAccelerationConstraint(7.5),
            new MaxAccelerationConstraint(30.0),
            new FeedforwardConstraint(MAX_FEEDFORWARD, KV, KA, true)
    };

    @Test
    public void speedTest() {
        final int speedRuns = 10;

        Logger logger = new Logger("TrajectoryTest.speedTest");

        Path loggedPath = new SplinePathBuilder()
                .addWaypoints(WAYPOINTS)
                .build();
        Trajectory loggedTrajectory = new Trajectory(loggedPath, CONSTRAINTS, SAMPLE_DISTANCE);

        MovingAverage average = new MovingAverage(speedRuns);

        double highTime = Double.NEGATIVE_INFINITY;
        double lowTime = Double.POSITIVE_INFINITY;

        for (int run = 1; run <= speedRuns; run++) {
            long start = System.nanoTime();

            Path path = new SplinePathBuilder()
                    .addWaypoints(WAYPOINTS)
                    .build();
            Trajectory trajectory = new Trajectory(path, CONSTRAINTS, SAMPLE_DISTANCE);

            int states = (int) Math.ceil(trajectory.getDuration() / DT);
            for (int i = 0; i <= states; i++) {
                trajectory.calculate(i * DT);
            }

            long end = System.nanoTime();

            double time = (end - start) * (Constants.MILLISECONDS / Constants.NANOSECONDS);

            if (time > highTime) {
                highTime = time;
            }
            if (time < lowTime) {
                lowTime = time;
            }
            average.add(time);
        }

        logger.info("Generated %d trajectories", speedRuns);
        logger.info("Trajectory duration: %.3f s", loggedTrajectory.getDuration());
        logger.info("Segments calculated for every %.3f ms", DT * Constants.MILLISECONDS);
        logger.info("Longest generation time: %.3f ms", highTime);
        logger.info("Shortest generation time: %.3f ms", lowTime);
        logger.info("Average trajectory time: %.3f ms", average.get());
    }

    @Test
    public void showGraph() throws IOException, PythonExecutionException {
        long startNs = System.nanoTime();

        Path splinePath = new SplinePathBuilder()
                .addWaypoints(WAYPOINTS)
                .build();

        Trajectory trajectory = new Trajectory(splinePath, CONSTRAINTS, 1.0e-2);

        int iterations = (int) (Math.ceil(trajectory.getDuration() / 5.0e-3));

        List<Double> recordedTimes = new ArrayList<>(iterations + 1);
        List<Double> recordedVelocities = new ArrayList<>(iterations + 1);
        List<Double> recordedAccelerations = new ArrayList<>(iterations + 1);
        List<Double> recordedDistances = new ArrayList<>(iterations + 1);
        List<Double> recordedFeedforwards = new ArrayList<>(iterations + 1);

        for (int i = 0; i <= iterations; i++) {
            double time = i * 5.0e-3;
            Trajectory.State state = trajectory.calculate(time);
            recordedTimes.add(time);
            recordedVelocities.add(state.getVelocity());
            recordedAccelerations.add(state.getAcceleration());
            recordedDistances.add(state.getPathState().getDistance());
            recordedFeedforwards.add(KV * state.getVelocity() + KA * state.getAcceleration());
        }
        long endNs = System.nanoTime();

        System.out.printf("Trajectory generation took %.3f ms!%n", (endNs - startNs) * 1.0e-6);


        Plot plt = Plot.create();
        plt.plot()
                .add(recordedTimes, recordedVelocities);
//        plt.plot()
//                .add(recordedTimes, recordedAccelerations);
//        plt.plot()
//                .add(recordedTimes, recordedDistances);
        plt.plot()
                .add(recordedTimes, recordedFeedforwards);

        plt.show();
    }
}
