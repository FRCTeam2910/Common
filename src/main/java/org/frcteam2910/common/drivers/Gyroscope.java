package org.frcteam2910.common.drivers;

import org.frcteam2910.common.math.MathUtils;

public abstract class Gyroscope {
	private double adjustmentAngle;
	private boolean inverted;

	public abstract void calibrate();

	public final double getAdjustmentAngle() {
		return adjustmentAngle;
	}

	public void setAdjustmentAngle(double adjustmentAngle) {
		this.adjustmentAngle = adjustmentAngle;
	}

	public final boolean isInverted() {
		return inverted;
	}

	public final void setInverted(boolean inverted) {
		this.inverted = inverted;
	}

	public abstract double getUnadjustedAngle();
	public abstract double getUnadjustedRate();

	public final double getAngle() {
		double angle = MathUtils.boundDegrees(getUnadjustedAngle() - adjustmentAngle);

		if (inverted) {
			return MathUtils.boundDegrees(-angle);
		}

		return angle;
	}

	public final double getRate() {
		double rate = getUnadjustedRate();

		if (inverted) {
			return -rate;
		}

		return rate;
	}
}
