package org.frcteam2910.common.control;

import org.frcteam2910.common.Constants;
import org.frcteam2910.common.Logger;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.MovingAverage;
import org.junit.Test;

import static org.hamcrest.Matchers.greaterThanOrEqualTo;
import static org.junit.Assert.assertThat;

public class TrajectoryTest {
    private static final double SAMPLE_DISTANCE = 1.0e-2;

    private static final double DT = 5.0e-3;

    private static final double KV = 0.8;
    private static final double KA = 0.1;
    private static final double MAX_FEEDFORWARD = 10.0;

    private static final double MAX_ACCELERATION = 30.0;
    private static final double ALLOWABLE_ACCELERATION_ERROR = 0.5;
    private static final double ALLOWABLE_VELOCITY_ERROR = 0.01;

    private static final TrajectoryConstraint[] CONSTRAINTS = {
            new CentripetalAccelerationConstraint(7.5),
            new MaxAccelerationConstraint(MAX_ACCELERATION),
            new FeedforwardConstraint(MAX_FEEDFORWARD, KV, KA, true)
    };

    @Test
    public void obeysConstraints() {
        Path path = new SplinePathBuilder(Vector2.ZERO, Rotation2.fromDegrees(90.0), Rotation2.fromDegrees(90.0))
                .hermite(new Vector2(50.0, 50.0), Rotation2.fromDegrees(90.0), Rotation2.ZERO)
                .build();
        Trajectory trajectory = new Trajectory(path, CONSTRAINTS, SAMPLE_DISTANCE);

        int samples = (int) Math.ceil(trajectory.getDuration() / DT);
        for (int i = 0; i <= samples; i++) {
            Trajectory.State currentState = trajectory.calculate(i * DT);

            double maximumVelocity = Double.POSITIVE_INFINITY;
            for (TrajectoryConstraint constraint : CONSTRAINTS) {
                maximumVelocity = Math.min(maximumVelocity, constraint.getMaxVelocity(currentState.getPathState()));
            }

            assertThat("Velocity exceeded maximum velocity", maximumVelocity + ALLOWABLE_VELOCITY_ERROR,
                    greaterThanOrEqualTo(Math.abs(currentState.getVelocity())));

            double maximumAcceleration = Double.POSITIVE_INFINITY;
            for (TrajectoryConstraint constraint : CONSTRAINTS) {
                if (currentState.getAcceleration() > 0.0) {
                    maximumAcceleration = Math.min(maximumAcceleration, constraint.getMaxAcceleration(currentState.getPathState(), currentState.getVelocity()));
                } else {
                    maximumAcceleration = Math.min(maximumAcceleration, constraint.getMaxDeceleration(currentState.getPathState(), currentState.getVelocity()));
                }
            }

            assertThat("Acceleration exceeded maximum acceleration", maximumAcceleration + ALLOWABLE_ACCELERATION_ERROR,
                    greaterThanOrEqualTo(Math.abs(currentState.getAcceleration())));
        }
    }

    @Test
    public void speedTest() {
        final int speedRuns = 10;

        Logger logger = new Logger("TrajectoryTest.speedTest");

        Path loggedPath = new SplinePathBuilder(Vector2.ZERO, Rotation2.fromDegrees(90.0), Rotation2.fromDegrees(90.0))
                .hermite(new Vector2(50.0, 50.0), Rotation2.fromDegrees(90.0), Rotation2.ZERO)
                .build();
        Trajectory loggedTrajectory = new Trajectory(loggedPath, CONSTRAINTS, SAMPLE_DISTANCE);

        MovingAverage average = new MovingAverage(speedRuns);

        double highTime = Double.NEGATIVE_INFINITY;
        double lowTime = Double.POSITIVE_INFINITY;

        for (int run = 1; run <= speedRuns; run++) {
            long start = System.nanoTime();

            Path path = new SplinePathBuilder(Vector2.ZERO, Rotation2.fromDegrees(90.0), Rotation2.fromDegrees(90.0))
                    .hermite(new Vector2(50.0, 50.0), Rotation2.fromDegrees(90.0), Rotation2.ZERO)
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
}
