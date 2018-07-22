package org.frcteam2910.common.math;

import java.text.DecimalFormat;
import java.util.Objects;

import static org.frcteam2910.common.math.MathUtils.epsilonEquals;

/**
 * A vector with 2 elements
 *
 * @since 0.2
 */
public final class Vector2 {
	public static final Vector2 ZERO = new Vector2(0, 0);

	public final double x, y;
	public final double length;

	/**
	 * Create a vector at (0, 0).
	 *
	 * @deprecated Use {@link Vector2#ZERO} instead.
	 */
	public Vector2() {
		this(0, 0);
	}

	public Vector2(double x, double y) {
		this.x = x;
		this.y = y;

		this.length = Math.hypot(x, y);
	}

	/**
	 * Copy a vector.
	 * @param other The vector to make a copy of.
	 *
	 * @deprecated Vectors are immutable, there is no reason to make copies.
	 */
	@Deprecated
	public Vector2(Vector2 other) {
		this.x = other.x;
		this.y = other.y;
		this.length = other.length;
	}

	/**
	 * Create a new unit vector with the specified angle.
	 * @param angle The angle of the new unit vector.
	 * @return A unit vector with the specified angle.
	 *
	 * @deprecated Use {@link #fromAngle(Rotation2)} instead.
	 */
	@Deprecated
	public static Vector2 fromAngle(double angle) {
		return new Vector2(Math.cos(angle), Math.sin(angle));
	}

	public static Vector2 fromAngle(Rotation2 rotation) {
		return new Vector2(rotation.cos, rotation.sin);
	}

	/**
	 * Calculate the angle between two vectors.
	 *
	 * @param a The vector that the angle begins at.
	 * @param b The vector that the angle ends at.
	 * @return The angle between the two vectors.
	 */
	public static Rotation2 getAngleBetween(Vector2 a, Vector2 b) {
		double cos = a.dot(b) / (a.length * b.length);
		if (Double.isNaN(cos)) {
			return Rotation2.ZERO;
		}

		return Rotation2.fromRadians(Math.acos(MathUtils.clamp(cos, -1.0, 1.0)));
	}

	public Rotation2 getAngle() {
		return new Rotation2(x, y, true);
	}

	public Vector2 add(Vector2 vector) {
		return add(vector.x, vector.y);
	}

	public Vector2 add(double x, double y) {
		return new Vector2(this.x + x, this.y + y);
	}

	public Vector2 subtract(Vector2 vector) {
		return subtract(vector.x, vector.y);
	}

	public Vector2 subtract(double x, double y) {
		return new Vector2(this.x - x, this.y - y);
	}

	/**
	 * Multiply each component of the vector by a scalar value.
	 * @param scalar The scalar to multiply each component by.
	 * @return The vector scaled by the scalar.
	 */
	public Vector2 scale(double scalar) {
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

	public double dot(Vector2 other) {
		return x * other.x + y * other.y;
	}

	public double cross(Vector2 other) {
		return x * other.y - y * other.x;
	}

	/**
	 * Rotate this vector by the specified rotation.
	 * @param rotation How much the vector should be rotated.
	 * @return The rotated vector.
	 */
	public Vector2 rotateBy(Rotation2 rotation) {
		return new Vector2(x * rotation.cos - y * rotation.sin, x * rotation.sin + y * rotation.cos);
	}

	@Override
	public boolean equals(Object obj) {
		if (!(obj instanceof Vector2)) {
			return false;
		}

		Vector2 other = (Vector2) obj;
		return epsilonEquals(x, other.x) && epsilonEquals(y, other.y);
	}

	@Override
	public int hashCode() {
		return Objects.hash(x, y);
	}

	@Override
	public String toString() {
		DecimalFormat fmt = new DecimalFormat("#0.000");
		return '(' + fmt.format(x) + ", " + fmt.format(y) + ')';
	}
}
