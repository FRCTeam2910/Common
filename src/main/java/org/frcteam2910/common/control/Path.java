package org.frcteam2910.common.control;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

public final class Path implements Serializable {
	private static final long serialVersionUID = 6482549382610337817L;

	private final List<PathSegment> segments = new ArrayList<>();
	private final List<Double> distancesFromStart = new ArrayList<>();

	private final InterpolatingTreeMap<InterpolatingDouble, Rotation2> rotationAtDistance = new InterpolatingTreeMap<>();

	private double length = 0.0;

	public Path(Rotation2 startRotation) {
	    rotationAtDistance.put(new InterpolatingDouble(0.0), startRotation);
    }

	public void addSegment(PathSegment segment) {
		segments.add(segment);
		distancesFromStart.add(length);
		length += segment.getLength();
	}

	public void addSegment(PathSegment segment, Rotation2 endRotation) {
	    addSegment(segment);
	    rotationAtDistance.put(new InterpolatingDouble(length), endRotation);
    }

	public List<PathSegment> getSegments() {
		return segments;
	}

	public double getDistanceToSegmentStart(int segment) {
		return distancesFromStart.get(segment);
	}

	public double getDistanceToSegmentEnd(int segment) {
		return distancesFromStart.get(segment) + segments.get(segment).getLength();
	}

	private int getSegmentAtDistance(double distance) {
		int start = 0;
		int end = segments.size() - 1;
		int mid = start + (end - start) / 2;

		while (start < end) {
			mid = start + (end - start) / 2;

			if (distance > getDistanceToSegmentEnd(mid)) {
				start = mid + 1;
			} else if (distance < getDistanceToSegmentStart(mid)) {
				end = mid;
			} else {
				break;
			}
		}

		return mid;
	}

	public Vector2 getPositionAtDistance(double distance) {
		int currentSegment = getSegmentAtDistance(distance);
		double segmentDistance = distance - getDistanceToSegmentStart(currentSegment);

		return segments.get(currentSegment).getPositionAtDistance(segmentDistance);
	}

	public Rotation2 getHeadingAtDistance(double distance) {
		int currentSegment = getSegmentAtDistance(distance);
		double segmentDistance = distance - getDistanceToSegmentStart(currentSegment);

		return segments.get(currentSegment).getHeadingAtDistance(segmentDistance);
	}

	public Rotation2 getRotationAtDistance(double distance) {
	    return rotationAtDistance.getInterpolated(new InterpolatingDouble(distance));
    }

	/**
	 * Subdivides the path by splitting each segment in half. This can be used to increase the resolution of a
	 * trajectory by allowing it to operate on more segments.
	 *
	 * @param iterations how many times to subdivide each segment
	 */
	public void subdivide(int iterations) {
		while (iterations-- > 0) {
			for (int i = 0; i < segments.size(); i++) {
				PathSegment seg = segments.get(i);

				PathSegment[] segs = seg.subdivide();

				segments.set(i, segs[0]);
				segments.add(++i, segs[1]);
				distancesFromStart.add(i, distancesFromStart.get(i - 1) + segs[0].getLength());
			}
		}
	}

	public double getLength() {
		return length;
	}
}
