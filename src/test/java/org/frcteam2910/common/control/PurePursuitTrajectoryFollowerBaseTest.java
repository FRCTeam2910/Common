package org.frcteam2910.common.control;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.junit.Test;

import static org.junit.Assert.assertTrue;

public class PurePursuitTrajectoryFollowerBaseTest {
    public static final double SEARCH_DT = 5.0e-3;

    public static final double ALLOWABLE_TRANSLATION_ERROR = 0.01;

    @Test
    public void findClosestSegmentTest() {
        Path path = new Path(Rotation2.ZERO);
        path.addSegment(new PathLineSegment(Vector2.ZERO, new Vector2(5.0, 0.0)), Rotation2.ZERO);
        Trajectory trajectory = new Trajectory(path,
                new MaxAccelerationConstraint(1.0),
                new MaxVelocityConstraint(1.0));

        Trajectory.Segment closest = PurePursuitTrajectoryFollowerBase.findClosestSegment(trajectory,
                new RigidTransform2(new Vector2(-1.0, 0.0), Rotation2.ZERO), SEARCH_DT, null);
        assertTrue(new Vector2(0.0, 0.0).equals(closest.translation, ALLOWABLE_TRANSLATION_ERROR));

        closest = PurePursuitTrajectoryFollowerBase.findClosestSegment(trajectory,
                new RigidTransform2(new Vector2(1.0, 0.5), Rotation2.ZERO), SEARCH_DT, null);
        assertTrue(new Vector2(1.0, 0.0).equals(closest.translation, ALLOWABLE_TRANSLATION_ERROR));
    }

    @Test
    public void findLookaheadSegmentTest() {
        Path path = new Path(Rotation2.ZERO);
        path.addSegment(new PathLineSegment(Vector2.ZERO, new Vector2(5.0, 0.0)), Rotation2.ZERO);
        Trajectory trajectory = new Trajectory(path,
                new MaxAccelerationConstraint(1.0),
                new MaxVelocityConstraint(1.0));

        Trajectory.Segment closest = PurePursuitTrajectoryFollowerBase.findClosestSegment(trajectory,
                new RigidTransform2(new Vector2(-1.0, 0.0), Rotation2.ZERO), SEARCH_DT, null);
        Trajectory.Segment lookahead = PurePursuitTrajectoryFollowerBase.findLookaheadSegment(trajectory,
                new RigidTransform2(new Vector2(0.0, 0.0), Rotation2.ZERO), 1.0,
                SEARCH_DT, closest);
        assertTrue(new Vector2(1.0, 0.0).equals(lookahead.translation, ALLOWABLE_TRANSLATION_ERROR));
    }
}
