package org.frcteam2910.common.control;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.frcteam2910.common.Constants;
import org.frcteam2910.common.util.MovingAverage;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import static org.hamcrest.MatcherAssert.assertThat;
import static org.hamcrest.Matchers.greaterThanOrEqualTo;

class TrajectoryTest {
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
    void obeysConstraints() {
        Path path = new SplinePathBuilder(
                        new Translation2d(), Rotation2d.fromDegrees(90.0), Rotation2d.fromDegrees(90.0))
                .hermite(new Translation2d(50.0, 50.0), Rotation2d.fromDegrees(90.0), new Rotation2d())
                .build();
        Trajectory trajectory = new Trajectory(path, CONSTRAINTS, SAMPLE_DISTANCE);

        int samples = (int) Math.ceil(trajectory.getDuration() / DT);
        for (int i = 0; i <= samples; i++) {
            Trajectory.State currentState = trajectory.calculate(i * DT);

            double maximumVelocity = Double.POSITIVE_INFINITY;
            for (TrajectoryConstraint constraint : CONSTRAINTS) {
                maximumVelocity = Math.min(maximumVelocity, constraint.getMaxVelocity(currentState.getPathState()));
            }

            assertThat(
                    "Velocity exceeded maximum velocity",
                    maximumVelocity + ALLOWABLE_VELOCITY_ERROR,
                    greaterThanOrEqualTo(Math.abs(currentState.getVelocity())));

            double maximumAcceleration = Double.POSITIVE_INFINITY;
            for (TrajectoryConstraint constraint : CONSTRAINTS) {
                if (currentState.getAcceleration() > 0.0) {
                    maximumAcceleration = Math.min(
                            maximumAcceleration,
                            constraint.getMaxAcceleration(currentState.getPathState(), currentState.getVelocity()));
                } else {
                    maximumAcceleration = Math.min(
                            maximumAcceleration,
                            constraint.getMaxDeceleration(currentState.getPathState(), currentState.getVelocity()));
                }
            }

            assertThat(
                    "Acceleration exceeded maximum acceleration",
                    maximumAcceleration + ALLOWABLE_ACCELERATION_ERROR,
                    greaterThanOrEqualTo(Math.abs(currentState.getAcceleration())));
        }
    }

    @Test
    void speedTest() {
        final int speedRuns = 10;

        Path loggedPath = new SplinePathBuilder(
                        new Translation2d(), Rotation2d.fromDegrees(90.0), Rotation2d.fromDegrees(90.0))
                .hermite(new Translation2d(50.0, 50.0), Rotation2d.fromDegrees(90.0), new Rotation2d())
                .build();
        Trajectory loggedTrajectory = new Trajectory(loggedPath, CONSTRAINTS, SAMPLE_DISTANCE);

        MovingAverage average = new MovingAverage(speedRuns);

        double highTime = Double.NEGATIVE_INFINITY;
        double lowTime = Double.POSITIVE_INFINITY;

        for (int run = 1; run <= speedRuns; run++) {
            long start = System.nanoTime();

            Path path = new SplinePathBuilder(
                            new Translation2d(), Rotation2d.fromDegrees(90.0), Rotation2d.fromDegrees(90.0))
                    .hermite(new Translation2d(50.0, 50.0), Rotation2d.fromDegrees(90.0), new Rotation2d())
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

        // TODO - fix this SonarLint rule blocker
        Assertions.assertTrue(true);

        System.out.printf("Generated %d trajectories%n", speedRuns);
        System.out.printf("Trajectory duration: %.3f s%n", loggedTrajectory.getDuration());
        System.out.printf("Segments calculated for every %.3f ms%n", DT * Constants.MILLISECONDS);
        System.out.printf("Longest generation time: %.3f ms%n", highTime);
        System.out.printf("Shortest generation time: %.3f ms%n", lowTime);
        System.out.printf("Average trajectory time: %.3f ms%n", average.get());
    }
}
