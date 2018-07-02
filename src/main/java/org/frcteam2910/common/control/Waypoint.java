package org.frcteam2910.common.control;

import com.team254.lib.util.math.Translation2d;

public class Waypoint {
	public final Translation2d position;
	public final double radius;

	public Waypoint(Translation2d position, double radius) {
		this.position = position;
		this.radius = radius;
	}

	public Waypoint(Waypoint other) {
		this(other.position, other.radius);
	}
}
