package org.frcteam2910.common.control;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.util.Optional;
import java.util.function.Function;

public abstract class PurePursuitTrajectoryFollowerBase<DriveSignalType> extends TrajectoryFollower<DriveSignalType> {
    public static final double DEFAULT_SEARCH_DT = 5.0e-3;

    private final Function<Vector2, Double> lookaheadDistanceFunction;
    private double finishRange;

    private double searchDt = DEFAULT_SEARCH_DT;
    private boolean finished = false;

    private Trajectory.Segment closestSegment = null;

    public PurePursuitTrajectoryFollowerBase(Function<Vector2, Double> lookaheadDistanceFunction,
                                             double finishRange) {
        this.lookaheadDistanceFunction = lookaheadDistanceFunction;
        this.finishRange = finishRange;
    }

    public PurePursuitTrajectoryFollowerBase(double lookaheadDistance, double finishRange) {
        this(velocity -> lookaheadDistance, finishRange);
    }

    static PathSegment getArcFromPointsAndHeading(Vector2 pointA, Rotation2 headingA, Vector2 pointB) {
        Vector2 abMidpoint = pointA.interpolate(pointB, 0.5);
        if (headingA.isParallel(abMidpoint.getAngle())) {
            if (headingA.equals(abMidpoint.getAngle())) {
                // Point A points exactly towards point B so we have a line.
                return new PathLineSegment(pointA, pointB);
            } else {
                // Point A points exactly away from point B so we cannot create an arc or a line to connect the two
                // points. We create a line from point A and hope the heading changes so we can create an actual arc.
                // The chance of this case happening is really low, so this should almost never happen.
                return new PathLineSegment(pointA, pointA.subtract(pointB.subtract(pointA)));
            }
        }

        Rotation2 midpointNormal = abMidpoint.getAngle().normal();

        Vector2 center;
        if (headingA.isParallel(midpointNormal)) {
            center = abMidpoint;
        } else {
            center = new RigidTransform2(pointA, headingA).intersection(new RigidTransform2(abMidpoint,
                    abMidpoint.getAngle().normal()));
        }

        return new PathArcSegment(pointA, pointB, center);
    }

    static Trajectory.Segment findClosestSegment(Trajectory trajectory, RigidTransform2 currentPose, double searchDt,
                                                 Trajectory.Segment startSegment) {
        if (startSegment == null) {
            startSegment = trajectory.calculateSegment(0.0);
        }

        Trajectory.Segment closestSegment = startSegment;
        double closestDistance = currentPose.translation.subtract(closestSegment.translation).length;
        boolean atEndOfTrajectory = false;
        while (!atEndOfTrajectory) {
            double searchTime = closestSegment.time + searchDt;

            Trajectory.Segment segment;

            // Use the last segment if the search time exceeds the trajectory's duration.
            if (searchTime >= trajectory.getDuration()) {
                segment = trajectory.calculateSegment(trajectory.getDuration());
                atEndOfTrajectory = true;
            } else {
                segment = trajectory.calculateSegment(searchTime);
            }

            // If the distance starts increasing we found "closest" segment
            double distance = currentPose.translation.subtract(segment.translation).length;
            if (distance > closestDistance) {
                break;
            }

            closestSegment = segment;
            closestDistance = distance;
        }

        return closestSegment;
    }

    static Trajectory.Segment findLookaheadSegment(Trajectory trajectory, RigidTransform2 currentPose,
                                                   double lookaheadDistance, double searchDt,
                                                   Trajectory.Segment startSegment) {
        Trajectory.Segment lookaheadSegment = startSegment;
        double actualLookaheadDistance = currentPose.translation.subtract(lookaheadSegment.translation).length;
        boolean atEndOfTrajectory = false;
        while (actualLookaheadDistance < lookaheadDistance && !atEndOfTrajectory) {
            double searchTime = lookaheadSegment.time + searchDt;

            // Use the last segment of the trajectory if the search time exceeds of the trajectory's duration.
            if (searchTime >= trajectory.getDuration()) {
                lookaheadSegment = trajectory.calculateSegment(trajectory.getDuration());
                atEndOfTrajectory = true;
            } else {
                lookaheadSegment = trajectory.calculateSegment(searchTime);
            }

            actualLookaheadDistance = currentPose.translation.subtract(lookaheadSegment.translation).length;
        }

        return lookaheadSegment;
    }

    protected abstract DriveSignalType getDriveSignal(RigidTransform2 currentPose, Vector2 velocity,
                                                      double rotationalVelocity, Trajectory trajectory,
                                                      Trajectory.Segment closestSegment,
                                                      Trajectory.Segment lookaheadSegment,
                                                      PathSegment pursuitSegment,
                                                      double dt);

    @Override
    protected DriveSignalType calculateDriveSignal(RigidTransform2 currentPose, Vector2 velocity,
                                                   double rotationalVelocity, Trajectory trajectory,
                                                   double time, double dt) {
        closestSegment = findClosestSegment(trajectory, currentPose, searchDt, closestSegment);

        Trajectory.Segment lookaheadSegment = findLookaheadSegment(trajectory, currentPose,
                lookaheadDistanceFunction.apply(velocity), searchDt, closestSegment);

        // If we are within the search time to the end of the trajectory we will say that we are finished.
        if (MathUtils.epsilonEquals(lookaheadSegment.time, trajectory.getDuration(), DEFAULT_SEARCH_DT)) {
            finished = true;
        }

        PathSegment pursuitSegment = getArcFromPointsAndHeading(currentPose.translation, velocity.getAngle(),
                lookaheadSegment.translation);

        return getDriveSignal(currentPose, velocity, rotationalVelocity, trajectory, closestSegment, lookaheadSegment,
                pursuitSegment, dt);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    protected void reset() {
        closestSegment = null;
        finished = false;
    }

    public double getFinishRange() {
        return finishRange;
    }

    public void setFinishRange(double finishRange) {
        this.finishRange = finishRange;
    }

    public double getSearchDt() {
        return searchDt;
    }

    public void setSearchDt(double searchDt) {
        this.searchDt = searchDt;
    }

    public Optional<Trajectory.Segment> getClosestSegment() {
        return Optional.ofNullable(closestSegment);
    }
}
