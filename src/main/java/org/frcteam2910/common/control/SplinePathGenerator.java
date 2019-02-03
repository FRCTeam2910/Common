package org.frcteam2910.common.control;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.math.spline.HermiteSpline;
import org.frcteam2910.common.math.spline.Spline;

import java.util.function.BiFunction;

/**
 * An implementation of a path generator that generates paths using splines.
 * <p>
 * The generator respects the following waypoint constraints:
 * <ul>
 *     <li>position</li>
 *     <li>heading</li>
 *     <li>rotation</li>
 * </ul>
 * <p>
 * To convert the generated splines into a series of arcs that can be followed in a path, the generator uses a
 * binary-search to fit arcs to each generated spline.
 */
public final class SplinePathGenerator implements PathGenerator {
    /**
     * The default maximum allowable positional error for an arc to be deemed good.
     */
    public static final double DEFAULT_FIT_CHECK_EPSILON = 1e-3;

    /**
     * The default amount of times the generator tries to fit an arc to a segment of each generated spline.
     */
    public static final int DEFAULT_FIT_TRIES = 25;

    /**
     * The allowable positional error for an arc to be considered good
     */
    private double fitCheckEpsilon = DEFAULT_FIT_CHECK_EPSILON;

    /**
     * The maximum amount of times an arc is fit to a segment of a spline before being "good enough"
     */
    private int fitTries = DEFAULT_FIT_TRIES;

    /**
     * Sets the allowable position error when checking if an arc fits on a spline. If the error is greater, the segment
     * is marked as bad and a smaller one is generated.
     * <p>
     * Increasing this value will result in faster path generation with the result of lowering generated path accuracy
     * when compared to just following each generated spline. The end position of the path is unaffected.
     *
     * @param fitCheckEpsilon the maximum positional error
     *
     * @see #DEFAULT_FIT_CHECK_EPSILON
     */
    public void setFitCheckEpsilon(double fitCheckEpsilon) {
        this.fitCheckEpsilon = fitCheckEpsilon;
    }

    /**
     * Sets the maximum amount of times the arc fitter attempts to fit a segment.
     * <p>
     * When the maximum amount of tries is reached, the fitter takes the segment as "good enough"
     *
     * @param fitTries the maximum amount of tries the fitter performs for the segment
     *
     * @see #DEFAULT_FIT_TRIES
     */
    public void setFitTries(int fitTries) {
        this.fitTries = fitTries;
    }

    /**
     * Generates a path using cubic splines.
     *
     * @param waypoints the waypoints to generate the path from
     * @return          the path generated from the waypoints
     *
     * @see #generate(BiFunction, Waypoint...)
     * @see HermiteSpline#cubic(RigidTransform2, RigidTransform2)
     */
    @Override
    public Path generate(Waypoint... waypoints) {
        return generate(HermiteSpline::cubic, waypoints);
    }

    /**
     * Generates a path using the specified spline-fitting algorithm.
     *
     * @param splineFunction a function that creates splines based on two positions and headings
     * @param waypoints      the waypoints to generate the path from
     * @return               the path generated from the waypoints
     */
    public Path generate(BiFunction<RigidTransform2, RigidTransform2, Spline> splineFunction, Waypoint... waypoints) {
        Path path = new Path(waypoints[0].rotation);

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
                PathSegment lastGoodSeg = null;

                PathSegment seg;

                int fitTry = 0;
                while (true) {
                    fitTry++;

                    double segDelta = segEnd - segStart;

                    double segMid = segStart + segDelta / 2.0;

                    Vector2 segStartPos = spline.getPoint(segStart);
                    Vector2 segEndPos = spline.getPoint(segEnd);
                    Vector2 segMidPos = spline.getPoint(segMid);

                    Vector2 deltaEnd = segEndPos.subtract(segStartPos);
                    Vector2 deltaMid = segMidPos.subtract(segStartPos);

                    if (Vector2.getAngleBetween(deltaMid, deltaEnd).equals(Rotation2.ZERO)) {
                        // The points form a line
                        seg = new PathLineSegment(segStartPos, segEndPos);
                    } else {
                        seg = PathArcSegment.fromPoints(segStartPos, segMidPos, segEndPos);
                    }

                    // If we are at the max tries, start the next arc
                    if (fitTry >= fitTries) {
                        break;
                    }

                    if (seg == null) {
                        segEnd -= segDelta / 2.0;
                        continue;
                    }

                    double segFirstQtr = segStart + segDelta / 4.0;
                    double segLastQtr = segStart + 3.0 * segDelta / 4.0;

                    // Make sure the points 25% and 75% through the segment fit
                    Vector2 approxFirstQtrPos = seg.getPositionAtPercentage(0.25);
                    Vector2 actualFirstQtrPos = spline.getPoint(segFirstQtr);
                    Vector2 approxLastQtrPos = seg.getPositionAtPercentage(0.75);
                    Vector2 actualLastQtrPos = spline.getPoint(segLastQtr);

                    Vector2 deltaFirstQtrPos = approxFirstQtrPos.subtract(actualFirstQtrPos);
                    Vector2 deltaLastQtrPos = approxLastQtrPos.subtract(actualLastQtrPos);

                    if (deltaFirstQtrPos.length > fitCheckEpsilon ||
                            deltaLastQtrPos.length > fitCheckEpsilon) {
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

                        // The end of the segment cannot be over 1. If it is, the results that the spline will calculate
                        // are undefined and could result in a lot of useless values at the end of the path.
                        segEnd = MathUtils.clamp(segEnd + segDelta / 2.0, 0.0, 1.0);
                    }
                }

                // Arc fits, move start and fit another arc
                if (MathUtils.epsilonEquals(1.0, segEnd)) {
                    path.addSegment(seg, end.rotation);
                } else {
                    path.addSegment(seg);
                }
                segStart = segEnd;
            }
        }

        return path;
    }
}
