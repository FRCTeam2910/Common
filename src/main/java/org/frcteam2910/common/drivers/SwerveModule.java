package org.frcteam2910.common.drivers;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Vector2;

/**
 * TODO: Document protected and private API
 */
public abstract class SwerveModule {
	private final Vector2 modulePosition;
	private double adjustmentAngle;

	private double previousAngle;
	private double previousDistance;
	private Vector2 currentPosition = new Vector2();

	private final double wheelDiameter;

	private boolean inverted = false;

	public SwerveModule(Vector2 modulePosition, double adjustmentAngle, double wheelDiameter) {
		this.modulePosition = new Vector2(modulePosition);
		this.adjustmentAngle = adjustmentAngle;
		this.wheelDiameter = wheelDiameter;
	}

	/**
	 * Get the amount of times the encoder on the angling motor has rotated.
	 *
	 * @return The amount of times the angle encoder has rotated.
	 */
	protected abstract double getAngleEncoderRotations();

	/**
	 * Get the amount of times the encoder on the driving motor has rotated.
	 *
	 * @return The amount of times the drive encoder has rotated.
	 */
	protected abstract double getDriveEncoderRotations();

	/**
	 * Get the drive encoder's rotations per second.
	 *
	 * @return The rotations per second of the drive encoder.
	 */
	protected abstract double getDriveEncoderRate();

	protected abstract void setTargetAngleRotations(double rotations);

	protected abstract void setDriveMotorInverted(boolean inverted);

    public abstract double getCurrentDrivePercentage();

    /**
     * Set the percentage of max power the module should drive at.
     *
     * @param percentage The percentage the module should drive at.
     */
    public abstract void setTargetDrivePercentage(double percentage);

    public abstract double getWheelScrubFactor();

	public abstract void zeroDistance();

	public final void setAdjustmentAngle(double adjustmentAngle) {
		this.adjustmentAngle = adjustmentAngle;
	}

	/**
	 * Get the distance the module is from the robot's center of rotation (usually the center).
	 *
	 * @return The distance the module is from the robot's center of rotation.
	 */
	public final Vector2 getModulePosition() {
		return modulePosition;
	}

	/**
	 * Get the current angle of the module in degrees.
	 *
	 * @return The current angle of the module.
	 */
	public final double getCurrentAngle() {
		double angle = getCurrentUnadjustedAngle();
		angle -= adjustmentAngle; // adjust angle
		angle %= 360;

		return angle;
	}

	/**
	 * Get the current angle of the module in degrees without the angle adjustment.
	 *
	 * @return The current, unadjusted angle of the module.
	 */
	public final double getCurrentUnadjustedAngle() {
		return getAngleEncoderRotations() * (360.0 / 1.0);
	}

	/**
	 * Get the distance the module has driven since the last reset in inches.
	 *
	 * @return The distance driven.
	 */
	public final double getCurrentDistance() {
		return getDriveEncoderRotations() * (Math.PI * wheelDiameter);
	}

	public final void setInverted(boolean inverted) {
		this.inverted = inverted;
	}

	public final boolean isInverted() {
		return inverted;
	}

	/**
	 * Set the angle the module should be at in degrees.
	 *
	 * @param targetAngle The target angle of the module.
	 */
	public final void setTargetAngle(double targetAngle) {
		targetAngle = MathUtils.boundDegrees(targetAngle + adjustmentAngle);

		double currentUnadjustedAngle = getCurrentUnadjustedAngle();
		double currentAngle = MathUtils.boundDegrees(currentUnadjustedAngle);

		double delta = currentAngle - targetAngle;

		if (delta > 180) {
			targetAngle += 360;
		} else if (delta < -180) {
			targetAngle -= 360;
		}

		delta = currentAngle - targetAngle;
		if (delta > 90 || delta < -90) {
			if (delta > 90) {
				targetAngle += 180;
			} else if (delta < -90) {
				targetAngle -= 180;
			}

			setDriveMotorInverted(false);
		} else {
			setDriveMotorInverted(true);
		}

		targetAngle += currentUnadjustedAngle - currentAngle;
		setTargetAngleRotations(targetAngle * (1.0 / 360.0));
	}

	public double getFieldCentricAngle(double robotHeading) {
		return Math.toDegrees(Vector2.fromAngle(Math.toRadians(getCurrentAngle())).rotateBy(Math.toRadians(robotHeading)).angle);
	}

	public synchronized final Vector2 getKinematicPosition() {
	    return currentPosition;
    }

	public synchronized final void updateKinematics(double heading) {
		heading = MathUtils.boundDegrees(heading);

		double currentDistance = getCurrentDistance();
		double deltaDistance = (currentDistance - previousDistance) * getWheelScrubFactor();
		double currentAngle = getFieldCentricAngle(heading);
		double averagedAngle = (currentAngle + previousAngle) / 2.0;

		Vector2 deltaPosition = Vector2.fromAngle(Math.toRadians(averagedAngle)).multiply(deltaDistance);
		currentPosition = currentPosition.add(deltaPosition);
		previousDistance = currentDistance;
		previousAngle = currentAngle;
	}

	public synchronized void resetKinematics(double heading) {
		heading = MathUtils.boundDegrees(heading);

		currentPosition = new Vector2();
		previousDistance = getCurrentDistance();
		previousAngle = getFieldCentricAngle(heading);
	}
}
