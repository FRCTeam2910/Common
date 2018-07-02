package org.frcteam2910.common.drivers;

import org.frcteam2910.common.math.MathUtils;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class GyroscopeTest {
	@Test
	public void adjustmentAngleTest() {
		MockGyroscope gyroscope = new MockGyroscope();
		gyroscope.setAdjustmentAngle(90.0);

		gyroscope.setUnadjustedAngle(90.0);
		assertEquals(0.0, gyroscope.getAngle(), MathUtils.EPSILON);

		gyroscope.setUnadjustedAngle(45.0);
		assertEquals(315.0, gyroscope.getAngle(), MathUtils.EPSILON);

		gyroscope.setUnadjustedAngle(100.0);
		assertEquals(10.0, gyroscope.getAngle(), MathUtils.EPSILON);
	}

	@Test
	public void invertedTest() {
		MockGyroscope gyroscope = new MockGyroscope();
		gyroscope.setInverted(true);

		gyroscope.setUnadjustedAngle(0.0);
		gyroscope.setUnadjustedRate(0.0);
		assertEquals(0.0, gyroscope.getAngle(), MathUtils.EPSILON);
		assertEquals(0.0, gyroscope.getRate(), MathUtils.EPSILON);

		gyroscope.setUnadjustedAngle(5.0);
		gyroscope.setUnadjustedRate(0.5);
		assertEquals(355.0, gyroscope.getAngle(), MathUtils.EPSILON);
		assertEquals(-0.5, gyroscope.getRate(), MathUtils.EPSILON);

		gyroscope.setUnadjustedAngle(15.0);
		gyroscope.setUnadjustedRate(-1.5);
		assertEquals(345.0, gyroscope.getAngle(), MathUtils.EPSILON);
		assertEquals(1.5, gyroscope.getRate(), MathUtils.EPSILON);
	}
}
