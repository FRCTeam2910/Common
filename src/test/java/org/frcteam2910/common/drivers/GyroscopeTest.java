package org.frcteam2910.common.drivers;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Rotation2;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class GyroscopeTest {
	@Test
	public void adjustmentAngleTest() {
		MockGyroscope gyroscope = new MockGyroscope();
		gyroscope.setAdjustmentAngle(Rotation2.fromDegrees(90.0));

		gyroscope.setUnadjustedAngle(Rotation2.fromDegrees(90.0));
		assertEquals(Rotation2.fromDegrees(0.0), gyroscope.getAngle());

		gyroscope.setUnadjustedAngle(Rotation2.fromDegrees(45.0));
		assertEquals(Rotation2.fromDegrees(315.0), gyroscope.getAngle());

		gyroscope.setUnadjustedAngle(Rotation2.fromDegrees(100.0));
		assertEquals(Rotation2.fromDegrees(10.0), gyroscope.getAngle());
	}

	@Test
	public void invertedTest() {
		MockGyroscope gyroscope = new MockGyroscope();
		gyroscope.setInverted(true);

		gyroscope.setUnadjustedAngle(Rotation2.fromDegrees(0.0));
		gyroscope.setUnadjustedRate(0.0);
		assertEquals(Rotation2.fromDegrees(0.0), gyroscope.getAngle());
		assertEquals(0.0, gyroscope.getRate(), MathUtils.EPSILON);

		gyroscope.setUnadjustedAngle(Rotation2.fromDegrees(5.0));
		gyroscope.setUnadjustedRate(0.5);
		assertEquals(Rotation2.fromDegrees(355.0), gyroscope.getAngle());
		assertEquals(-0.5, gyroscope.getRate(), MathUtils.EPSILON);

		gyroscope.setUnadjustedAngle(Rotation2.fromDegrees(15.0));
		gyroscope.setUnadjustedRate(-1.5);
		assertEquals(Rotation2.fromDegrees(345.0), gyroscope.getAngle());
		assertEquals(1.5, gyroscope.getRate(), MathUtils.EPSILON);
	}
}
