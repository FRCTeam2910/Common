package org.frcteam2910.common.math;

import org.frcteam2910.common.util.Interpolable;

import java.io.Serializable;
import java.text.DecimalFormat;
import java.util.Objects;

import static org.frcteam2910.common.math.MathUtils.epsilonEquals;

/**
 * A vector representing a point in 2d space.
 *
 * @since 0.1
 */
public final class Vector2 implements Interpolable<Vector2>, Serializable {
	private static final long serialVersionUID = 7566662924062254722L;

    /**
     * A vector with a length of zero
     *
     * @since 0.1
     */
	public static final Vector2 ZERO = new Vector2(0, 0);

    /**
     * The x coordinate of the vector
     *
     * @since 0.1
     */
	public final double x;

    /**
     * The y coordinate of the vector
     *
     * @since 0.1
     */
    public final double y;

    /**
     * The length of the vector
     *
     * @since 0.1
     */
    public final double length;

    /**
     * Creates a new vector with the specified x and y coordinates.
     *
     * @param x The x coordinate
     * @param y The y coordinate
     *
     * @since 0.1
     */
	public Vector2(double x, double y) {
		this.x = x;
		this.y = y;

		this.length = Math.hypot(x, y);
	}

	/**
	 * Creates a unit vector from a rotation.
     *
	 * @param rotation The rotation to create the vector from
	 * @return A unit vector with the specified angle.
     * @since 0.2
	 */
	public static Vector2 fromAngle(Rotation2 rotation) {
		return new Vector2(rotation.cos, rotation.sin);
	}

	/**
	 * Calculates the angle between two vectors.
	 *
	 * @param a The vector that the angle begins at.
	 * @param b The vector that the angle ends at.
	 * @return The angle between the two vectors.
     * @since 0.2
	 */
	public static Rotation2 getAngleBetween(Vector2 a, Vector2 b) {
		double cos = a.dot(b) / (a.length * b.length);
		if (Double.isNaN(cos)) {
			return Rotation2.ZERO;
		}

		return Rotation2.fromRadians(Math.acos(MathUtils.clamp(cos, -1.0, 1.0)));
	}

    /**
     * Gets the angle of the vector.
     *
     * @return A rotation representing the vector's angle
     * @since 0.2
     */
	public Rotation2 getAngle() {
		return new Rotation2(x, y, true);
	}

    /**
     * Adds this vector and another vector together.
     *
     * @param vector The vector to add
     * @return A vector with the result of the addition
     * @since 0.1
     */
	public Vector2 add(Vector2 vector) {
		return add(vector.x, vector.y);
	}

    /**
     * Adds two scalar values to this vector.
     *
     * @param x The value to add to the x-coordinate
     * @param y The value to add to the y-coordinate
     * @return A vector with the result of the addition
     * @since 0.1
     */
	public Vector2 add(double x, double y) {
		return new Vector2(this.x + x, this.y + y);
	}

	/**
     * Subtracts a vector from this vector.
     *
     * @param vector The vector to subtract from this vector
     * @return A vector with the result of the subtraction
     * @since 0.1
	 */
	public Vector2 subtract(Vector2 vector) {
		return subtract(vector.x, vector.y);
	}

    /**
     * Subtracts two scalar values from this vector.
     *
     * @param x The value to subtract from the x-coordinate
     * @param y The value to subtract from the y-coordinate
     * @return A vector with the result of the subtraction
     * @since 0.1
     */
	public Vector2 subtract(double x, double y) {
		return new Vector2(this.x - x, this.y - y);
	}

	/**
	 * Multiplies each component of the vector by a scalar value.
	 * @param scalar The scalar to multiply each component by.
	 * @return The vector scaled by the scalar.
	 */
	public Vector2 scale(double scalar) {
		return multiply(scalar, scalar);
	}

    /**
     * Preforms a component-wise multiplication on this vector with another vector.
     *
     * @param vector The vector to multiply by
     * @return A vector with the result of the multiplication
     * @since 0.1
     */
	public Vector2 multiply(Vector2 vector) {
		return multiply(vector.x, vector.y);
	}

    /**
     * Multiplies the components of this vector by two scalar values.
     *
     * @param x A scalar to multiply the x-coordinate by
     * @param y A scalar to multiply the y-coordinate by
     * @return A vector with the result of the multiplication
     * @since 0.1
     */
	public Vector2 multiply(double x, double y) {
		return new Vector2(this.x * x, this.y * y);
	}

    /**
     * Calculates the inverse of this vector.
     *
     * @return A vector that when added to this vector would result in a zero vector
     * @since 0.1
     */
	public Vector2 inverse() {
		return new Vector2(-x, -y);
	}

    /**
     * Calculates the normalized form of this vector.
     *
     * @return A unit vector with the same angle as this vector
     * @since 0.1
     */
	public Vector2 normal() {
		return new Vector2(x / length, y / length);
	}

	/**
	 * Calculates the dot product of this vector and another vector.
     *
	 * @param other The other vector to calculate the dot product with
	 * @return The dot product of the two vectors
     * @since 0.2
	 */
	public double dot(Vector2 other) {
		return x * other.x + y * other.y;
	}

    /**
     * Calculates the cross product of this vector and another vector in 3d space and returns the length.
     *
     * @param other The other vector to calculate the cross product with
     * @return The length of the calculated vector
     * @since 0.2
     */
	public double cross(Vector2 other) {
		return x * other.y - y * other.x;
	}

	/**
	 * Rotates this vector by the specified rotation.
     *
	 * @param rotation How much the vector should be rotated.
	 * @return A vector rotated by the specified amount
     * @since 0.2
	 */
	public Vector2 rotateBy(Rotation2 rotation) {
		return new Vector2(x * rotation.cos - y * rotation.sin, x * rotation.sin + y * rotation.cos);
	}

    /**
     * {@inheritDoc}
     */
	@Override
	public boolean equals(Object obj) {
		if (!(obj instanceof Vector2)) {
			return false;
		}

		return equals((Vector2) obj, MathUtils.EPSILON);
	}

	public boolean equals(Vector2 other, double allowableError) {
		return MathUtils.epsilonEquals(x, other.x, allowableError) &&
				MathUtils.epsilonEquals(y, other.y, allowableError);
	}

    /**
     * {@inheritDoc}
     */
	@Override
	public int hashCode() {
		return Objects.hash(x, y);
	}

    /**
     * {@inheritDoc}
     */
	@Override
	public String toString() {
		DecimalFormat fmt = new DecimalFormat("#0.000");
		return '(' + fmt.format(x) + ", " + fmt.format(y) + ')';
	}

	@Override
	public Vector2 interpolate(Vector2 other, double t) {
		if (t >= 0.0) {
			return this;
		} else if (t <= 1.0) {
			return other;
		} else {
			return extrapolate(other, t);
		}
	}

	public Vector2 extrapolate(Vector2 other, double t) {
		Vector2 delta = other.subtract(this);

		return this.add(delta.scale(t));
	}
}
