package org.frcteam2910.common.drivers;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import static org.junit.Assert.assertTrue;

public final class MockSwerveModule extends SwerveModule {
	private double angleRotations;
	private double targetAngleRotations;
	private double driveDistance;
	private double driveRate;
	private double targetDrivePercentage;
	private boolean driveInverted;

	public MockSwerveModule(Vector2 modulePosition, Rotation2 adjustmentAngle) {
		super(modulePosition, adjustmentAngle);
	}

	@Override
	protected double getAngleEncoderRotations() {
		return angleRotations;
	}

	public void setAngleRotations(double angleRotations) {
		this.angleRotations = angleRotations;
	}

	@Override
	public double getCurrentDistance() {
		return driveDistance;
	}

	public void setCurrentDistance(double driveEncoderRotations) {
		this.driveDistance = driveEncoderRotations;
	}

	@Override
	public double getCurrentRate() {
		return driveRate;
	}

	public void setCurrentRate(double driveEncoderRate) {
		this.driveRate = driveEncoderRate;
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
		this.driveDistance = 0;
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
		driveDistance += driveRate * deltaTime;
	}
}
