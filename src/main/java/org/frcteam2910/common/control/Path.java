package org.frcteam2910.common.control;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.util.ArrayList;
import java.util.List;

public class Path {
	private final List<Segment> segments = new ArrayList<>();
	private final List<Double> distancesFromStart = new ArrayList<>();

	private double length = 0.0;

	public void addSegment(Segment segment) {
		segments.add(segment);
		distancesFromStart.add(length);
		length += segment.getLength();
	}

	public List<Segment> getSegments() {
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

	public Rotation2 getSlopeAtDistance(double distance) {
		int currentSegment = getSegmentAtDistance(distance);
		double segmentDistance = distance - getDistanceToSegmentStart(currentSegment);

		return segments.get(currentSegment).getSlopeAtDistance(segmentDistance);
	}

	public double getLength() {
		return length;
	}

	public abstract static class Segment {
		public abstract Vector2 getStart();

		public abstract Vector2 getEnd();

		public Vector2 getPositionAtDistance(double distance) {
			return getPositionAtPercentage(distance / getLength());
		}

		public abstract Vector2 getPositionAtPercentage(double percentage);

		public Rotation2 getSlopeAtDistance(double distance) {
			return getSlopeAtPercentage(distance / getLength());
		}

		public abstract Rotation2 getSlopeAtPercentage(double percentage);

		public abstract double getLength();

		public static class Line extends Segment {
			private final Vector2 start, end;
			private final Vector2 delta;

			public Line(Vector2 start, Vector2 end) {
				this.start = start;
				this.end = end;
				this.delta = end.subtract(start);
			}

			@Override
			public Vector2 getStart() {
				return start;
			}

			@Override
			public Vector2 getEnd() {
				return end;
			}

			@Override
			public Vector2 getPositionAtPercentage(double percentage) {
				return start.add(delta.scale(percentage));
			}

			@Override
			public Rotation2 getSlopeAtPercentage(double percentage) {
				return delta.getAngle();
			}

			@Override
			public double getLength() {
				return delta.length;
			}
		}

		public static class Arc extends Segment {
			private final Vector2 start, end, center;
			private final Vector2 deltaStart, deltaEnd;

			public Arc(Vector2 start, Vector2 end, Vector2 center) {
				this.start = start;
				this.end = end;
				this.center = center;
				this.deltaStart = start.subtract(center);
				this.deltaEnd = end.subtract(center);
			}


			public static Arc fromPoints(Vector2 a, Vector2 b, Vector2 c) {
				Line chordAB = new Line(a, b);
				Line chordBC = new Line(b, c);

				RigidTransform2 perpChordAB = new RigidTransform2(chordAB.getPositionAtPercentage(0.5), chordAB.getSlopeAtPercentage(0.5).normal());
				RigidTransform2 perpChordBC = new RigidTransform2(chordBC.getPositionAtPercentage(0.5), chordBC.getSlopeAtPercentage(0.5).normal());

				Vector2 center = perpChordAB.intersection(perpChordBC);

				// TODO: Check if the arc goes the long way around the circle.

				return new Arc(a, c, center);
			}

			@Override
			public Vector2 getStart() {
				return start;
			}

			@Override
			public Vector2 getEnd() {
				return end;
			}

			@Override
			public Vector2 getPositionAtPercentage(double percentage) {
				double deltaAngle = Vector2.getAngleBetween(deltaStart, deltaEnd).toRadians() *
						((deltaStart.cross(deltaEnd) >= 0) ? 1 : -1) * percentage;
				return center.add(deltaStart.rotateBy(Rotation2.fromRadians(deltaAngle)));
			}

			@Override
			public Rotation2 getSlopeAtPercentage(double percentage) {
				double angle = Vector2.getAngleBetween(deltaStart, deltaEnd).toRadians() * percentage;
				return deltaStart.rotateBy(Rotation2.fromRadians(angle)).getAngle().normal();
			}

			@Override
			public double getLength() {
				return deltaStart.length * Vector2.getAngleBetween(deltaStart, deltaEnd).toRadians();
			}

			public double getRadius() {
				return deltaStart.length;
			}
		}
	}
}
