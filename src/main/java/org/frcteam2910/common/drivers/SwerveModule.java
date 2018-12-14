package org.frcteam2910.common.drivers;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

/**
 * TODO: Document protected and private API
 */
public abstract class SwerveModule {
	private final Vector2 modulePosition;
	private Rotation2 adjustmentAngle;

	private double previousDistance;
	private Vector2 currentPosition = Vector2.ZERO;
	private String name = "Unknown";

	public SwerveModule(Vector2 modulePosition, Rotation2 adjustmentAngle) {
		this.modulePosition = modulePosition;
		this.adjustmentAngle = adjustmentAngle;
	}

	public void setName(String name) {
		this.name = name;
	}

	public String getName() {
		return name;
	}

	/**
	 * Get the amount of times the encoder on the angling motor has rotated.
	 *
	 * @return The amount of times the angle encoder has rotated.
	 */
	protected abstract double getAngleEncoderRotations();

	protected abstract void setTargetAngleRotations(double rotations);

	protected abstract void setDriveMotorInverted(boolean inverted);

    public abstract double getCurrentDrivePercentage();

    /**
     * Set the percentage of max power the module should drive at.
     *
     * @param percentage The percentage the module should drive at.
     */
    public abstract void setTargetDrivePercentage(double percentage);

	public abstract void zeroDistance();

	public final void setAdjustmentAngle(Rotation2 adjustmentAngle) {
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
	public final Rotation2 getCurrentAngle() {
		return getCurrentUnadjustedAngle().rotateBy(adjustmentAngle.inverse());
	}

	/**
	 * Get the current angle of the module in degrees without the angle adjustment.
	 *
	 * @return The current, unadjusted angle of the module.
	 */
	public final Rotation2 getCurrentUnadjustedAngle() {
		return Rotation2.fromRadians(getAngleEncoderRotations() * 2 * Math.PI);
	}

	/**
	 * Get the distance the module has driven since the last reset in inches.
	 *
	 * @return The distance driven.
	 */
	public abstract double getCurrentDistance();

	/**
	 * Get the rate that the module is driving in inches per second.
	 * @return The rate in inches per second.
	 */
	public abstract double getCurrentRate();

	/**
	 * Set the angle the module should be at in degrees.
	 *
	 * @param rotation The target angle of the module.
	 */
	public final void setTargetAngle(Rotation2 rotation) {
		double targetAngle = rotation.toDegrees() + adjustmentAngle.toDegrees() % 360;
		if (targetAngle < 0.0) {
			targetAngle += 360.0;
		}

		double currentUnadjustedAngle = getAngleEncoderRotations() * 360.0;
		double currentAngle = currentUnadjustedAngle % 360;
		if (currentAngle < 0.0) {
			currentAngle += 360.0;
		}

		double delta = currentAngle - targetAngle;
		if (delta > 180.0) {
			targetAngle += 360.0;
		} else if (delta < -180.0) {
			targetAngle -= 360.0;
		}

		delta = currentAngle - targetAngle;
		if (delta > 90.0 || delta < -90.0) {
			if (delta > 90) {
				targetAngle += 180.0;
			} else if (delta < -90) {
				targetAngle -= 180.0;
			}

			setDriveMotorInverted(true);
		} else {
			setDriveMotorInverted(false);
		}

		targetAngle += currentUnadjustedAngle - currentAngle;
		setTargetAngleRotations(targetAngle * (1.0 / 360.0));
	}

	public Rotation2 getFieldCentricAngle(Rotation2 robotHeading) {
		return getCurrentAngle().rotateBy(robotHeading);
	}

	public synchronized final Vector2 getKinematicPosition() {
	    return currentPosition;
    }

	public synchronized final void updateKinematics(Rotation2 heading) {
		double currentDistance = getCurrentDistance();
		double deltaDistance = (currentDistance - previousDistance);
		Rotation2 currentAngle = getFieldCentricAngle(heading);

		Vector2 deltaPosition = Vector2.fromAngle(currentAngle).scale(deltaDistance);

		currentPosition = currentPosition.add(deltaPosition);
		previousDistance = currentDistance;
	}

	public synchronized void resetKinematics(Rotation2 heading) {
		currentPosition = Vector2.ZERO;
		previousDistance = 0;
	}
}
