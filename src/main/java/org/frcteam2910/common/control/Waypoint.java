package org.frcteam2910.common.control;

import org.frcteam2910.common.math.Vector2;

public class Waypoint {
	public final Vector2 position;
	public final double radius;

	public Waypoint(Vector2 position, double radius) {
		this.position = position;
		this.radius = radius;
	}

	public Waypoint(Waypoint other) {
		this(other.position, other.radius);
	}
}
