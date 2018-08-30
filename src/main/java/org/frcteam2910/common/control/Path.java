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

		return segments.get(currentSegment).getHeadingAtDistance(segmentDistance);
	}

	public double getLength() {
		return length;
	}

	public abstract static class Segment {
		private final RigidTransform2 start;
		private final RigidTransform2 end;

		public Segment(RigidTransform2 start, RigidTransform2 end) {
			this.start = start;
			this.end = end;
		}

		public final RigidTransform2 getStart() {
			return start;
		}

		public final RigidTransform2 getEnd() {
			return end;
		}

		public Vector2 getPositionAtDistance(double distance) {
			return getPositionAtPercentage(distance / getLength());
		}

		public abstract Vector2 getPositionAtPercentage(double percentage);

		public Rotation2 getHeadingAtDistance(double distance) {
			return getHeadingAtPercentage(distance / getLength());
		}

		public abstract Rotation2 getHeadingAtPercentage(double percentage);

		public Rotation2 getRotationAtDistance(double distance) {
			return getRotationAtPercentage(distance / getLength());
		}

		public Rotation2 getRotationAtPercentage(double percentage) {
			return start.rotation.interpolate(end.rotation, percentage);
		}

		public abstract double getLength();

		public static class Line extends Segment {
			private final Vector2 start, end;
			private final Vector2 delta;

			public Line(RigidTransform2 start, RigidTransform2 end) {
				super(start, end);
				this.start = start.translation;
				this.end = end.translation;
				this.delta = end.translation.subtract(start.translation);
			}

			@Override
			public Vector2 getPositionAtPercentage(double percentage) {
				return start.add(delta.scale(percentage));
			}

			@Override
			public Rotation2 getHeadingAtPercentage(double percentage) {
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

			public Arc(RigidTransform2 start, RigidTransform2 end, Vector2 center) {
				super(start, end);
				this.start = start.translation;
				this.end = end.translation;
				this.center = center;
				this.deltaStart = start.translation.subtract(center);
				this.deltaEnd = end.translation.subtract(center);
			}

			public static Arc fromPoints(Vector2 a, Vector2 b, Vector2 c, Rotation2 startRotation, Rotation2 endRotation) {
				Line chordAB = new Line(new RigidTransform2(a, Rotation2.ZERO), new RigidTransform2(b, Rotation2.ZERO));
				Line chordBC = new Line(new RigidTransform2(b, Rotation2.ZERO), new RigidTransform2(c, Rotation2.ZERO));

				RigidTransform2 perpChordAB = new RigidTransform2(chordAB.getPositionAtPercentage(0.5), chordAB.getHeadingAtPercentage(0.5).normal());
				RigidTransform2 perpChordBC = new RigidTransform2(chordBC.getPositionAtPercentage(0.5), chordBC.getHeadingAtPercentage(0.5).normal());

				Vector2 center = perpChordAB.intersection(perpChordBC);

				// TODO: Check if the arc goes the long way around the circle.

				return new Arc(new RigidTransform2(a, startRotation), new RigidTransform2(c, endRotation), center);
			}

			@Override
			public Vector2 getPositionAtPercentage(double percentage) {
				double deltaAngle = Vector2.getAngleBetween(deltaStart, deltaEnd).toRadians() *
						((deltaStart.cross(deltaEnd) >= 0) ? 1 : -1) * percentage;
				return center.add(deltaStart.rotateBy(Rotation2.fromRadians(deltaAngle)));
			}

			@Override
			public Rotation2 getHeadingAtPercentage(double percentage) {
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
