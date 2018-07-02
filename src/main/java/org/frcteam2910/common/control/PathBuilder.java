package org.frcteam2910.common.control;

import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;

public class PathBuilder {
	private static final double EPSILON = 1e-9;
	private static final double REALLY_BIG_NUMBER = 1e9;

	public static Path build(Waypoint... waypoints) {
		Path path = new Path();

		if (waypoints.length < 2)
			throw new IllegalArgumentException("Path must contain at least 2 waypoints");

		int i = 0;
		while (i < waypoints.length - 2) {
			new Arc(waypoints[i], waypoints[i + 1], waypoints[i + 2]).addToPath(path);
			i++;
		}
		new Line(waypoints[i], waypoints[i + 1]).addToPath(path);

		return path;
	}

	private static class Line {
		private final Waypoint a;
		private final Waypoint b;
		private final Translation2d start;
		private final Translation2d end;
		private final Translation2d slope;

		public Line(Waypoint a, Waypoint b) {
			this.a = a;
			this.b = b;
			slope = new Translation2d(a.position, b.position);
			start = a.position.translateBy(slope.scale(a.radius / slope.norm()));
			end = b.position.translateBy(slope.scale(-b.radius / slope.norm()));
		}

		private void addToPath(Path p) {
			double pathLength = new Translation2d(end, start).norm();
			if (pathLength > EPSILON) {
					p.addSegment(new Path.Segment.Line(start, end));
			}

		}
	}

	private static class Arc {
		private final Line ab;
		private final Line bc;
		private final Translation2d center;
		private final double radius;

		public Arc(Waypoint a, Waypoint b, Waypoint c) {
			this(new Line(a, b), new Line(b, c));
		}

		public Arc(Line ab, Line bc) {
			this.ab = ab;
			this.bc = bc;
			this.center = intersect(ab, bc);
			this.radius = new Translation2d(center, ab.end).norm();
		}

		private void addToPath(Path p) {
			ab.addToPath(p);
			if (radius > EPSILON && radius < REALLY_BIG_NUMBER) {
				p.addSegment(new Path.Segment.Arc(ab.end, bc.start, center));
			}
		}

		private static Translation2d intersect(Line l1, Line l2) {
			final RigidTransform2d lineA = new RigidTransform2d(l1.end, new Rotation2d(l1.slope, true).normal());
			final RigidTransform2d lineB = new RigidTransform2d(l2.start, new Rotation2d(l2.slope, true).normal());
			return lineA.intersection(lineB);
		}
	}
}
