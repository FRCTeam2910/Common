package org.frcteam2910.common.drivers;

import org.frcteam2910.common.math.Rotation2;

public final class MockGyroscope extends Gyroscope {
	private Rotation2 unadjustedAngle;
	private double unadjustedRate;

	@Override
	public void calibrate() { }

	@Override
	public Rotation2 getUnadjustedAngle() {
		return unadjustedAngle;
	}

	public void setUnadjustedAngle(Rotation2 unadjustedAngle) {
		this.unadjustedAngle = unadjustedAngle;
	}

	@Override
	public double getUnadjustedRate() {
		return unadjustedRate;
	}

	public void setUnadjustedRate(double unadjustedRate) {
		this.unadjustedRate = unadjustedRate;
	}
}
