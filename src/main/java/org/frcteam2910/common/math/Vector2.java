package org.frcteam2910.common.math;

import static org.frcteam2910.common.math.MathUtils.epsilonEquals;

/**
 * A vector with 2 elements
 */
public final class Vector2 {
	public static final Vector2 ZERO = new Vector2(0, 0);

	public final double x, y;
	public final double length;
	public final double angle;

	public Vector2() {
		this(0, 0);
	}

	public Vector2(double x, double y) {
		this.x = x;
		this.y = y;

		this.length = Math.hypot(x, y);

		this.angle = Math.atan2(y, x);
	}

	public Vector2(Vector2 other) {
		this.x = other.x;
		this.y = other.y;
		this.length = other.length;
		this.angle = other.angle;
	}

	public static Vector2 fromAngle(double angle) {
		return new Vector2(Math.cos(angle), Math.sin(angle));
	}

	public Vector2 add(double scalar) {
		return add(scalar, scalar);
	}

	public Vector2 add(Vector2 vector) {
		return add(vector.x, vector.y);
	}

	public Vector2 add(double x, double y) {
		return new Vector2(this.x + x, this.y + y);
	}

	public Vector2 subtract(double scalar) {
		return subtract(scalar, scalar);
	}

	public Vector2 subtract(Vector2 vector) {
		return subtract(vector.x, vector.y);
	}

	public Vector2 subtract(double x, double y) {
		return new Vector2(this.x - x, this.y - y);
	}

	public Vector2 multiply(double scalar) {
		return multiply(scalar, scalar);
	}

	public Vector2 multiply(Vector2 vector) {
		return multiply(vector.x, vector.y);
	}

	public Vector2 multiply(double x, double y) {
		return new Vector2(this.x * x, this.y * y);
	}

	public Vector2 inverse() {
		return new Vector2(-x, -y);
	}

	public Vector2 normalize() {
		return new Vector2(x / length, y / length);
	}

	public Vector2 rotateBy(double radians) {
		double cos = Math.cos(radians);
		double sin = Math.sin(radians);

		return new Vector2(x * cos - y * sin, x * sin + y * cos);
	}

	@Override
	public boolean equals(Object obj) {
		if (!(obj instanceof Vector2))
			return false;
		Vector2 other = (Vector2) obj;
		return epsilonEquals(x, other.x) && epsilonEquals(y, other.y);
	}

	@Override
	public String toString() {
		return String.format("(%.3f, %.3f)", x, y);
	}
}
