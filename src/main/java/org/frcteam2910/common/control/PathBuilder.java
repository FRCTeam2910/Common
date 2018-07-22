package org.frcteam2910.common.control;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Vector2;

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
		private final Vector2 start;
		private final Vector2 end;
		private final Vector2 delta;

		public Line(Waypoint a, Waypoint b) {
			this.a = a;
			this.b = b;
			delta = b.position.subtract(a.position);
			start = a.position.add(delta.scale(a.radius / delta.length));
			end = b.position.add(delta.scale(-b.radius / delta.length));
		}

		private void addToPath(Path p) {
			double pathLength = delta.length;
			if (pathLength > EPSILON) {
				p.addSegment(new Path.Segment.Line(start, end));
			}
		}
	}

	private static class Arc {
		private final Line ab;
		private final Line bc;
		private final Vector2 center;
		private final double radius;

		public Arc(Waypoint a, Waypoint b, Waypoint c) {
			this(new Line(a, b), new Line(b, c));
		}

		public Arc(Line ab, Line bc) {
			this.ab = ab;
			this.bc = bc;
			this.center = intersect(ab, bc);
			this.radius = ab.end.subtract(center).length;
		}

		private void addToPath(Path p) {
			ab.addToPath(p);
			if (radius > EPSILON && radius < REALLY_BIG_NUMBER) {
				p.addSegment(new Path.Segment.Arc(ab.end, bc.start, center));
			}
		}

		private static Vector2 intersect(Line l1, Line l2) {
			final RigidTransform2 lineA = new RigidTransform2(l1.end, l1.delta.getAngle().normal());
			final RigidTransform2 lineB = new RigidTransform2(l2.start, l2.delta.getAngle().normal());
			return lineA.intersection(lineB);
		}
	}
}
