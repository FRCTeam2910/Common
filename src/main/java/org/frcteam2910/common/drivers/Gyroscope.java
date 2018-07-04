package org.frcteam2910.common.drivers;

import org.frcteam2910.common.math.Rotation2;

public abstract class Gyroscope {
	private Rotation2 adjustmentAngle = Rotation2.ZERO;
	private boolean inverted;

	public abstract void calibrate();

	public final Rotation2 getAdjustmentAngle() {
		return adjustmentAngle;
	}

	public void setAdjustmentAngle(Rotation2 adjustmentAngle) {
		this.adjustmentAngle = adjustmentAngle;
	}

	public final boolean isInverted() {
		return inverted;
	}

	public final void setInverted(boolean inverted) {
		this.inverted = inverted;
	}

	public abstract Rotation2 getUnadjustedAngle();
	public abstract double getUnadjustedRate();

	public final Rotation2 getAngle() {
		Rotation2 angle = getUnadjustedAngle().rotateBy(adjustmentAngle.inverse());

		if (inverted) {
			return angle.inverse();
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
