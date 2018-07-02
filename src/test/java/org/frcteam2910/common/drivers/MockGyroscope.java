package org.frcteam2910.common.drivers;

public final class MockGyroscope extends Gyroscope {
	private double unadjustedAngle;
	private double unadjustedRate;

	@Override
	public void calibrate() { }

	@Override
	public double getUnadjustedAngle() {
		return unadjustedAngle;
	}

	public void setUnadjustedAngle(double unadjustedAngle) {
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
