package org.frcteam2910.common.control;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.math.spline.HermiteSpline;
import org.frcteam2910.common.math.spline.Spline;

import java.util.function.BiFunction;

public final class SplinePathGenerator implements PathGenerator {
    public static final double DEFAULT_FIT_CHECK_EPSILON = 1e-3;
    public static final int DEFAULT_FIT_TRIES = 25;

    private double fitCheckEpsilon = DEFAULT_FIT_CHECK_EPSILON;
    private int fitTries = DEFAULT_FIT_TRIES;

    public void setFitCheckEpsilon(double fitCheckEpsilon) {
        this.fitCheckEpsilon = fitCheckEpsilon;
    }

    public void setFitTries(int fitTries) {
        this.fitTries = fitTries;
    }

    @Override
    public Path generate(Waypoint... waypoints) {
        return generate(HermiteSpline::cubic, waypoints);
    }

    public Path generate(BiFunction<RigidTransform2, RigidTransform2, Spline> splineFunction, Waypoint... waypoints) {
        Path path = new Path();

        for (int i = 1; i < waypoints.length; i++) {
            Waypoint start = waypoints[i - 1];
            Waypoint end = waypoints[i];

            RigidTransform2 startPose = new RigidTransform2(start.position, start.heading);
            RigidTransform2 endPose = new RigidTransform2(end.position, end.heading);

            Spline spline = splineFunction.apply(startPose, endPose);

            double segStart = 0.0;
            while (!MathUtils.epsilonEquals(segStart, 1.0)) {
                double segEnd = 1.0;

                double lastGoodEnd = segEnd;
                Path.Segment lastGoodSeg = null;

                Path.Segment seg;

                int fitTry = 0;
                while (true) {
                    fitTry++;

                    double segDelta = segEnd - segStart;

                    double segMid = segStart + segDelta / 2.0;

                    Vector2 segStartPos = spline.getPoint(segStart);
                    Vector2 segEndPos = spline.getPoint(segEnd);
                    Vector2 segMidPos = spline.getPoint(segMid);

                    if (Vector2.getAngleBetween(segStartPos, segMidPos).equals(Rotation2.ZERO) &&
                            Vector2.getAngleBetween(segMidPos, segEndPos).equals(Rotation2.ZERO)) {
                        // The points form a line
                        seg = new Path.Segment.Line(segStartPos, segEndPos);
                    } else {
                        seg = Path.Segment.Arc.fromPoints(segStartPos, segMidPos, segEndPos);
                    }

                    // If we are at the max tries, start the next arc
                    if (fitTry >= fitTries) {
                        break;
                    }

                    double segFirstQtr = segStart + segDelta / 4.0;
                    double segLastQtr = segStart + 3.0 * segDelta / 4.0;

                    // Make sure the points 25% and 75% through the segment fit
                    Vector2 approxFirstQtrPos = seg.getPositionAtPercentage(0.25);
                    Vector2 actualFirstQtrPos = spline.getPoint(segFirstQtr);
                    Vector2 approxLastQtrPos = seg.getPositionAtPercentage(0.75);
                    Vector2 actualLastQtrPos = spline.getPoint(segLastQtr);

                    if (approxFirstQtrPos.subtract(actualFirstQtrPos).length > fitCheckEpsilon ||
                            approxLastQtrPos.subtract(actualLastQtrPos).length > fitCheckEpsilon) {
                        // Half delta and try again
                        segEnd -= segDelta / 2.0;
                        if (lastGoodSeg != null) {
                            // Boundary found, use last good segment
                            seg = lastGoodSeg;
                            segEnd = lastGoodEnd;

                            break;
                        }
                    } else {
                        // The segment is good
                        lastGoodSeg = seg;
                        lastGoodEnd = segEnd;

                        segEnd += segDelta / 2.0;
                    }
                }

                // Arc fits, move start and fit another arc
                path.addSegment(seg);
                segStart = segEnd;
            }
        }

        return path;
    }
}
