package org.frcteam2910.common.control;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;

import java.util.ArrayList;
import java.util.List;

public final class Path {
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

	private double getDistanceToSegmentStart(int segment) {
		return distancesFromStart.get(segment);
	}

	private double getDistanceToSegmentEnd(int segment) {
		return distancesFromStart.get(segment) + segments.get(segment).getLength();
	}

	private int getSegmentAtDistance(double distance) {
		for (int i = 0; i < segments.size() - 1; i++) {
			double beginDistance = getDistanceToSegmentStart(i);
			double endDistance = getDistanceToSegmentEnd(i);

			if (beginDistance <= distance && distance < endDistance)
				return i;
		}

		return segments.size() - 1;
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

	public double getLength() {
		return length;
	}
}
