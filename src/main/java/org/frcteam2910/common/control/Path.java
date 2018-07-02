package org.frcteam2910.common.control;

import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class Path {
	private final List<Segment> segments = new ArrayList<>();

	public void addSegment(Segment segment) {
		segments.add(segment);
	}

	public List<Segment> getSegments() {
		return segments;
	}

	private double getDistanceToSegmentStart(int segment) {
		double distance = 0;
		for (int i = 0; i < segment; i++)
			distance += segments.get(i).getLength();
		return distance;
	}

	private double getDistanceToSegmentEnd(int segment) {
		double distance = 0;
		for (int i = 0; i <= segment; i++)
			distance += segments.get(i).getLength();
		return distance;
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

	public Translation2d getPositionAtDistance(double distance) {
		int currentSegment = getSegmentAtDistance(distance);
		double segmentDistance = distance - getDistanceToSegmentStart(currentSegment);

		return segments.get(currentSegment).getPositionAtDistance(segmentDistance);
	}

	public Rotation2d getSlopeAtDistance(double distance) {
		int currentSegment = getSegmentAtDistance(distance);
		double segmentDistance = distance - getDistanceToSegmentStart(currentSegment);

		return segments.get(currentSegment).getSlopeAtDistance(segmentDistance);
	}

	public double getLength() {
		double length = 0;
		for (int i = 0; i < segments.size(); i++)
			length += segments.get(i).getLength();
		return length;
	}

	public abstract static class Segment {
		public abstract Translation2d getStart();

		public abstract Translation2d getEnd();

		public Translation2d getPositionAtDistance(double distance) {
			return getPositionAtPercentage(distance / getLength());
		}

		public abstract Translation2d getPositionAtPercentage(double percentage);

		public Rotation2d getSlopeAtDistance(double distance) {
			return getSlopeAtPercentage(distance / getLength());
		}

		public abstract Rotation2d getSlopeAtPercentage(double percentage);

		public abstract double getLength();

		public static class Line extends Segment {
			private final Translation2d start, end;
			private final Translation2d delta;

			public Line(Translation2d start, Translation2d end) {
				this.start = start;
				this.end = end;
				this.delta = new Translation2d(start, end);
			}

			@Override
			public Translation2d getStart() {
				return start;
			}

			@Override
			public Translation2d getEnd() {
				return end;
			}

			@Override
			public Translation2d getPositionAtPercentage(double percentage) {
				return start.translateBy(delta.scale(percentage));
			}

			@Override
			public Rotation2d getSlopeAtPercentage(double percentage) {
				return delta.direction();
			}

			@Override
			public double getLength() {
				return delta.norm();
			}
		}

		public static class Arc extends Segment {
			private final Translation2d start, end, center;
			private final Translation2d deltaStart, deltaEnd;

			public Arc(Translation2d start, Translation2d end, Translation2d center) {
				this.start = start;
				this.end = end;
				this.center = center;
				this.deltaStart = new Translation2d(center, start);
				this.deltaEnd = new Translation2d(center, end);
			}

			@Override
			public Translation2d getStart() {
				return start;
			}

			@Override
			public Translation2d getEnd() {
				return end;
			}

			@Override
			public Translation2d getPositionAtPercentage(double percentage) {
				double deltaAngle = Translation2d.getAngle(deltaStart, deltaEnd).getRadians() *
						((Translation2d.cross(deltaStart, deltaEnd) >= 0) ? 1 : -1) * percentage;
				return center.translateBy(deltaStart.rotateBy(Rotation2d.fromRadians(deltaAngle)));
			}

			@Override
			public Rotation2d getSlopeAtPercentage(double percentage) {
				double angle = Translation2d.getAngle(deltaStart, deltaEnd).getRadians() * percentage;
				return deltaStart.rotateBy(Rotation2d.fromRadians(angle)).direction().normal();
			}

			@Override
			public double getLength() {
				return deltaStart.norm() * Translation2d.getAngle(deltaStart, deltaEnd).getRadians();
			}

			public double getRadius() {
				return deltaStart.norm();
			}
		}
	}
}
