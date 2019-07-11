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
import static org.junit.Assert.*;

public class TrajectoryTest {
    public static final int SPEED_RUNS = 10;
    public static final double DT = 5 / Constants.MILLISECONDS;

    private static final double KV = 0.8;
    private static final double KA = 0.1;
    private static final double KS = 0.0;

    private static final double MAX_ACCELERATION = 5.0;
    private static final double ALLOWABLE_ACCELERATION_ERROR = 0.01;

    private static final double MAX_FEEDFORWARD = 12.0;
    private static final double ALLOWABLE_FEEDFORWARD_ERROR = 0.5;

    private static final double ALLOWABLE_DISTANCE_ERROR = 0.05;
    private static final double ALLOWABLE_HEADING_ERROR = Math.toRadians(0.05);
    private static final double ALLOWABLE_POSITION_ERROR = 0.05;
    private static final double ALLOWABLE_ROTATION_ERROR = Math.toRadians(0.05);

    private static final Waypoint[] WAYPOINTS = {
            new Waypoint(new Vector2(0, 0), Rotation2.fromDegrees(90), Rotation2.fromDegrees(90)),
            new Waypoint(new Vector2(50, 50), Rotation2.fromDegrees(90), Rotation2.fromDegrees(0))
    };
    private static final Path[] PATHS;

    private static final ITrajectoryConstraint[] CONSTRAINTS = {
            new MaxAccelerationConstraint(MAX_ACCELERATION),
            new CentripetalAccelerationConstraint(1.0),
            new FeedforwardConstraint(MAX_FEEDFORWARD, KV, KA, KS)
    };

    static {
        Path simplePath = new Path(Rotation2.ZERO);
        simplePath.addSegment(new PathLineSegment(Vector2.ZERO, new Vector2(50, 0)), Rotation2.ZERO);
        simplePath.subdivide(16);

        Path curvedPath = new Path(Rotation2.ZERO);
        curvedPath.addSegment(new PathLineSegment(Vector2.ZERO, new Vector2(50, 0)));
        curvedPath.addSegment(new PathArcSegment(new Vector2(50, 0), new Vector2(100, 50), new Vector2(50, 50)));
        curvedPath.addSegment(new PathLineSegment(new Vector2(100, 50), new Vector2(100, 100)));
        curvedPath.subdivide(8);

        SplinePathGenerator generator = new SplinePathGenerator();
        Path splinePath = generator.generate(WAYPOINTS);
        splinePath.subdivide(2);

        PATHS = new Path[]{
                simplePath,
                curvedPath,
                splinePath
        };
    }

    private static double calculateFeedforward(Trajectory.Segment segment) {
        return KV * segment.velocity + KA * segment.acceleration + Math.copySign(KS, segment.velocity);
    }

    @Test
    public void feedforwardTest() {
        for (int i = 0; i < PATHS.length; i++) {
            Path path = PATHS[i];
            Trajectory trajectory = new Trajectory(path, CONSTRAINTS);

            for (int j = 0; j < Math.ceil(trajectory.getDuration() / DT); j++) {
                Trajectory.Segment segment = trajectory.calculateSegment(j * DT);

                double absFeedforward = Math.abs(calculateFeedforward(segment));

                assertThat("Actual feedforward exceeds max feedforward", absFeedforward,
                        lessThanOrEqualTo(MAX_FEEDFORWARD + ALLOWABLE_FEEDFORWARD_ERROR));
            }
        }
    }

    @Test
    public void startEndTest() {
        for (Path path : PATHS) {
            Trajectory trajectory = new Trajectory(path, CONSTRAINTS);

            Vector2 pathStart = path.getPositionAtDistance(0.0);
            Vector2 pathEnd = path.getPositionAtDistance(path.getLength());
            Trajectory.Segment trajStart = trajectory.calculateSegment(0.0);
            Trajectory.Segment trajEnd = trajectory.calculateSegment(trajectory.getDuration());

            assertEquals("Starting velocity is not zero", 0.0, trajStart.velocity, MathUtils.EPSILON);
            assertEquals("Start X values do not match", pathStart.x, trajStart.translation.x, MathUtils.EPSILON);
            assertEquals("Start Y values do not match", pathStart.y, trajStart.translation.y, MathUtils.EPSILON);
            assertEquals("Ending velocity is not zero", 0.0, trajEnd.velocity, MathUtils.EPSILON);
            assertEquals("End X values do not match", pathEnd.x, trajEnd.translation.x, MathUtils.EPSILON);
            assertEquals("End Y values do not match", pathEnd.y, trajEnd.translation.y, MathUtils.EPSILON);
        }
    }

    @Test
    public void velocityContinuityTest() {
        for (Path path : PATHS) {
            Trajectory trajectory = new Trajectory(path, CONSTRAINTS);

            Trajectory.Segment previousSegment = null;
            for (int i = 0; i < Math.ceil(trajectory.getDuration() / DT); i++) {
                Trajectory.Segment segment = trajectory.calculateSegment(i * DT);
                if (previousSegment == null) {
                    previousSegment = segment;
                    continue;
                }

                double absAcceleration = Math.abs(previousSegment.velocity - segment.velocity) / DT;

                assertThat("Actual acceleration exceeds max acceleration", absAcceleration,
                        lessThanOrEqualTo(MAX_ACCELERATION + ALLOWABLE_ACCELERATION_ERROR));

                previousSegment = segment;
            }
        }
    }

    @Test
    public void verifyStartAndEndTest() {
        Path path = new Path(Rotation2.ZERO);
        path.addSegment(new PathLineSegment(Vector2.ZERO, new Vector2(12.0, 0.0)), Rotation2.ZERO);
        path.subdivide(10);
        Trajectory trajectory = new Trajectory(path,
                new ITrajectoryConstraint() {
                    @Override
                    public double getMaxVelocity(PathSegment segment) {
                        return 2.0;
                    }

                    @Override
                    public double getMaxAcceleration(PathSegment segment, double velocity) {
                        return 1.0;
                    }
                });

        Trajectory.Segment segment = trajectory.calculateSegment(0.0);

        assertEquals("Starting distance does not match the desired distance",
                0.0, segment.position, ALLOWABLE_DISTANCE_ERROR);
        assertTrue(String.format("Starting heading (%s) does not match the desired heading (%s)",
                segment.heading, Rotation2.ZERO),
                segment.heading.equals(Rotation2.ZERO, ALLOWABLE_HEADING_ERROR));
        assertEquals("Starting X position does not match the desired X position",
                0.0, segment.translation.x, ALLOWABLE_POSITION_ERROR);
        assertEquals("Starting Y position does not match the desired Y position",
                0.0, segment.translation.y, ALLOWABLE_POSITION_ERROR);
        assertTrue(String.format("Starting rotation (%s) does not match the desired rotation (%s)",
                segment.rotation, Rotation2.ZERO),
                segment.rotation.equals(Rotation2.ZERO, ALLOWABLE_ROTATION_ERROR));

        segment = trajectory.calculateSegment(trajectory.getDuration());

        assertEquals("Ending distance does not match the desired distance",
                12.0, segment.position, ALLOWABLE_DISTANCE_ERROR);
        assertTrue(String.format("Ending heading (%s) does not match the desired heading (%s)",
                segment.heading, Rotation2.ZERO),
                segment.heading.equals(Rotation2.ZERO, ALLOWABLE_HEADING_ERROR));
        assertEquals("Ending X position does not match the desired X Position",
                12.0, segment.translation.x, ALLOWABLE_POSITION_ERROR);
        assertEquals("Ending Y position does not match the desired Y position",
                0.0, segment.translation.y, ALLOWABLE_POSITION_ERROR);
        assertTrue(String.format("Ending rotation (%s) does not match the desired rotation (%s)",
                segment.rotation, Rotation2.ZERO),
                segment.rotation.equals(Rotation2.ZERO, ALLOWABLE_ROTATION_ERROR));
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
            trajectory.calculateSegments(DT);

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
        logger.info("Segments calculated for every %.3f ms", DT * Constants.MILLISECONDS);
        logger.info("Longest generation time: %.3f ms", highTime);
        logger.info("Shortest generation time: %.3f ms", lowTime);
        logger.info("Average trajectory time: %.3f ms", average.get());
    }

    @Test
    @Ignore
    public void writeCsv() {
        Trajectory trajectory = new Trajectory(0.0, 4.0, PATHS[1], CONSTRAINTS);

        try (PrintStream out = new PrintStream(new FileOutputStream("trajectory.csv"))) {
            out.printf("segment,time,x,y,heading,rotation,position,velocity,acceleration,maxVelocity,f%n");
            for (int i = 0; i < Math.ceil(trajectory.getDuration() / DT); i++) {
                Trajectory.Segment segment = trajectory.calculateSegment(i * DT);
                out.printf("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f%n", segment.pathSegmentIndex, segment.time, segment.translation.x,
                        segment.translation.y, segment.heading.toDegrees(), segment.rotation.toDegrees(), segment.position,
                        segment.velocity, segment.acceleration, segment.maxVelocity, calculateFeedforward(segment));
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
