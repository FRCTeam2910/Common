package org.frcteam2910.common.math;

import static java.lang.Math.abs;

public class MathUtils {
	public static final double EPSILON = 1e-9;

	public static boolean epsilonEquals(double a, double b) {
		return epsilonEquals(a, b, EPSILON);
	}

	public static boolean epsilonEquals(double a, double b, double epsilon) {
		return abs(a - b) < epsilon;
	}

	public static double limit(double v, double maxMagnitude) {
		return limit(v, -maxMagnitude, maxMagnitude);
	}

	public static double limit(double v, double min, double max) {
		return Math.min(max, Math.max(v, min));
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

	public static boolean isInRange(double lowerBound, double upperBound, double value) {
		return lowerBound <= value && value <= upperBound;
	}
}
