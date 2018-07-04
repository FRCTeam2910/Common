package org.frcteam2910.common.drivers;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

/**
 * TODO: Document protected and private API
 */
public abstract class SwerveModule {
	private final Vector2 modulePosition;
	private Rotation2 adjustmentAngle;

	private Rotation2 previousAngle = Rotation2.ZERO;
	private double previousDistance;
	private Vector2 currentPosition = Vector2.ZERO;
	private String name = "Unknown";

	private boolean inverted = false;

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

    public abstract double getWheelScrubFactor();

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
	public final void setTargetAngle(Rotation2 targetAngle) {
		Rotation2 currentAngle = getCurrentAngle();

		Rotation2 delta = currentAngle.rotateBy(targetAngle.inverse());
		double deltaDegrees = delta.toDegrees();
		if (MathUtils.isInRange(90, 270, deltaDegrees)) {
			if (MathUtils.isInRange(90, 180, deltaDegrees)) {
				targetAngle = targetAngle.rotateBy(Rotation2.fromDegrees(180));
			} else if (MathUtils.isInRange(180, 270, deltaDegrees)) {
				targetAngle = targetAngle.rotateBy(Rotation2.fromDegrees(-180));
			}

			setDriveMotorInverted(true);
		} else {
			setDriveMotorInverted(false);
		}

		int moduleRotations;
		double currentEncoderRotations = getAngleEncoderRotations();
		if (currentEncoderRotations > 0) {
			moduleRotations = (int) Math.floor(currentEncoderRotations);
		} else {
			moduleRotations = (int) Math.ceil(currentEncoderRotations);
		}

		setTargetAngleRotations((targetAngle.rotateBy(adjustmentAngle).toDegrees()) / 360.0 + moduleRotations);
	}

	public Rotation2 getFieldCentricAngle(Rotation2 robotHeading) {
		return getCurrentAngle().rotateBy(robotHeading);
	}

	public synchronized final Vector2 getKinematicPosition() {
	    return currentPosition;
    }

	public synchronized final void updateKinematics(Rotation2 heading) {
		double currentDistance = getCurrentDistance();
		double deltaDistance = (currentDistance - previousDistance) * getWheelScrubFactor();
		Rotation2 currentAngle = getFieldCentricAngle(heading);
		Rotation2 averagedAngle = Rotation2.fromRadians((currentAngle.toRadians() + previousAngle.toRadians()) / 2.0);

		Vector2 deltaPosition = Vector2.fromAngle(averagedAngle).scale(deltaDistance);
		if (inverted) {
			deltaPosition = deltaPosition.inverse();
		}

		currentPosition = currentPosition.add(deltaPosition);
		previousDistance = currentDistance;
		previousAngle = currentAngle;
	}

	public synchronized void resetKinematics(Rotation2 heading) {
		currentPosition = Vector2.ZERO;
		previousDistance = 0;
		previousAngle = getFieldCentricAngle(heading);
	}
}
