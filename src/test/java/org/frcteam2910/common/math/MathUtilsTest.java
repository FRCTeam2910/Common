package org.frcteam2910.common.math;

import org.junit.Test;

import static org.frcteam2910.common.math.MathUtils.*;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class MathUtilsTest {
	@Test
	public void boundDegreesTest() {
		assertEquals(15.0, boundDegrees(15.0), EPSILON);
		assertEquals(270.0, boundDegrees(-90.0), EPSILON);
		assertEquals(180.0, boundDegrees(540.0), EPSILON);
	}

	@Test
	public void boundRadiansTest() {
		assertEquals(Math.PI, boundRadians(Math.PI), EPSILON);
		assertEquals(Math.PI, boundRadians(3 * Math.PI), EPSILON);
		assertEquals(Math.PI / 4, boundRadians(9 * Math.PI / 4), EPSILON);
		assertEquals(7 * Math.PI / 4, boundRadians(-Math.PI / 4), EPSILON);
	}

	@Test
	public void epsilonEqualsTest() {
		assertTrue(epsilonEquals(1.0, 1.0));
		assertFalse(epsilonEquals(1.0, 1.0001, 0.00001));
		assertTrue(epsilonEquals(1.0, 1.05, 0.1));
		assertTrue(epsilonEquals(-1.0 / 2.0, -0.5));
	}

	@Test
	public void isInRangeTest() {
		assertTrue(isInRange(0.0, 1.0, 0.5));
		assertTrue(isInRange(-1.0, 1.0, 1.0));
		assertFalse(isInRange(-1.0, 5.0, 10.0));
		assertFalse(isInRange(-5.0, 2.5, -5.1));
	}

	@Test
	public void limitTest() {
		assertEquals(5.0, limit(5.0, 10.0), EPSILON);
		assertEquals(1.0, limit(5.0, 1.0), EPSILON);
		assertEquals(-2.5, limit(-3.0, -2.5, 5.0), EPSILON);
		assertEquals(1.0, limit(1.0, -2.5, 5.0), EPSILON);
	}
}
