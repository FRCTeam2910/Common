package org.frcteam2910.common.drivers;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Vector2;

import static org.junit.Assert.assertTrue;

public final class MockSwerveModule extends SwerveModule {
	private double angle;
	private double distance;
	private double targetAngle;
	private double driveOutput;

	public MockSwerveModule(Vector2 modulePosition) {
		super(modulePosition);
	}

	@Override
	protected double readAngle() {
		return angle;
	}

	public void writeAngle(double angle) {
		this.angle = angle;
	}

	@Override
	public double readDistance() {
		return distance;
	}

	public void writeDistance(double distance) {
		this.distance = distance;
	}

	@Override
	protected void setTargetAngle(double angle) {
		this.targetAngle = angle;
	}

	public double getTargetAngle() {
		return targetAngle;
	}

	@Override
	public void setDriveOutput(double output) {
		assertTrue("Drive output of swerve module is not in the range [-1, 1]",
				MathUtils.isInRange(-1.0, 1.0, output));

		this.driveOutput = output;
	}

	public double getDriveOutput() {
		return driveOutput;
	}
}
