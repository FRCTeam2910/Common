package org.frcteam2910.common.math;

import static java.lang.Math.abs;

/**
 * Miscellaneous math functions
 *
 * @since 0.1
 */
public class MathUtils {
	public static final double EPSILON = 1e-9;

	/**
	 * Check if two numbers are equal to each other using the default epsilon.
	 *
	 * @param a The first number.
	 * @param b The second number.
	 *
	 * @return If the two numbers are equal.
	 *
	 * @since 0.1
	 */
	public static boolean epsilonEquals(double a, double b) {
		return epsilonEquals(a, b, EPSILON);
	}

	/**
	 * Check if two numbers are equal to each other.
	 *
	 * @param a The first number
	 * @param b The second number
	 * @param epsilon The epsilon the comparison will use. Think of this as the allowable difference between the two
	 *                numbers
	 *
	 * @return If the numbers are equal.
	 *
	 * @since 0.1
	 */
	public static boolean epsilonEquals(double a, double b, double epsilon) {
		return abs(a - b) < epsilon;
	}

	/**
	 * Clamps a value between a minimum and a maximum value.
	 *
	 * @param value The value to clamp.
	 * @param min The minimum value of the range. This value must be less than max.
	 * @param max The maximum value of the range. This value must be less than min.
	 * @return the clamped value.
	 *
	 * @since 0.2
	 */
	public static double clamp(double value, double min, double max) {
		if (min > max) {
			throw new IllegalArgumentException("min must not be greater than max");
		}

		return Math.max(min, Math.min(value, max));
	}

	public static double boundDegrees(double angle) {
		angle %= 360;
		if (angle < 0)
			angle += 360;
		return angle;
	}

	public static double boundRadians(double angle) {
		angle %= 2 * Math.PI;
		if (angle < 0)
			angle += 2 * Math.PI;
		return angle;
	}

	/**
	 * Returns if the value is in the range [lowerBound, upperBound].
	 *
	 * @param lowerBound The lower bound of the range.
	 * @param upperBound The upper bound of the range.
	 * @param value The value.
	 *
	 * @return If the value is in the range.
	 *
	 * @since 0.1
	 */
	public static boolean isInRange(double lowerBound, double upperBound, double value) {
		return lowerBound <= value && value <= upperBound;
	}
}
