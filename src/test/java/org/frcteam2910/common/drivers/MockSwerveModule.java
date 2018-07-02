package org.frcteam2910.common.drivers;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Vector2;

import static org.junit.Assert.assertTrue;

public final class MockSwerveModule extends SwerveModule {
	private double angleRotations;
	private double targetAngleRotations;
	private double driveRotations;
	private double driveRotationRate;
	private double targetDrivePercentage;
	private boolean driveInverted;

	public MockSwerveModule(Vector2 modulePosition, double adjustmentAngle, double wheelDiameter) {
		super(modulePosition, adjustmentAngle, wheelDiameter);
	}

	@Override
	protected double getAngleEncoderRotations() {
		return angleRotations;
	}

	public void setAngleRotations(double angleRotations) {
		this.angleRotations = angleRotations;
	}

	@Override
	protected double getDriveEncoderRotations() {
		return driveRotations;
	}

	public void setDriveRotations(double driveEncoderRotations) {
		this.driveRotations = driveEncoderRotations;
	}

	@Override
	protected double getDriveEncoderRate() {
		return driveRotationRate;
	}

	public void setDriveRotationRate(double driveEncoderRate) {
		this.driveRotationRate = driveEncoderRate;
	}

	public double getTargetAngleRotations() {
		return targetAngleRotations;
	}

	@Override
	protected void setTargetAngleRotations(double rotations) {
		this.targetAngleRotations = rotations;
	}

	public boolean isDriveInverted() {
		return driveInverted;
	}

	@Override
	protected void setDriveMotorInverted(boolean inverted) {
		this.driveInverted = inverted;
	}

	@Override
	public void zeroDistance() {
		this.driveRotations = 0;
	}

	@Override
	public double getCurrentDrivePercentage() {
		return targetDrivePercentage;
	}

	public double getTargetDrivePercentage() {
		return targetDrivePercentage;
	}

	@Override
	public void setTargetDrivePercentage(double percentage) {
		assertTrue("Target percentage of swerve module is not in the range [-1, 1]",
				MathUtils.isInRange(-1, 1, percentage));

		this.targetDrivePercentage = percentage;
	}

	@Override
	public double getWheelScrubFactor() {
		return 1;
	}

	public void accumulateRates(double deltaTime) {
		driveRotations += driveRotationRate * deltaTime;
	}
}
