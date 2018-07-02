package org.frcteam2910.common.robot.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.frcteam2910.common.Constants;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.drivers.SwerveModule;

import static org.frcteam2910.common.robot.Constants.CAN_TIMEOUT_MS;

/**
 * Driver for the 2017 revision of the 2910 swerve module
 */
public final class SwerveModule2017 extends SwerveModule {
	private static final double SCRUB_FACTOR = 1.0; // TODO: Figure out actual scrub factor (Take from 1323?)

	private static final double TICKS_PER_INCH = 36.65; // TODO: Find actual number

	private final TalonSRX angleMotor, driveMotor;

	public SwerveModule2017(Vector2 modulePosition,
	                        double adjustmentAngle,
	                        int angleMotor,
	                        int driveMotor) {
		super(modulePosition, adjustmentAngle);
		this.angleMotor = new TalonSRX(angleMotor);
		this.driveMotor = new TalonSRX(driveMotor);

		// Configure angle motor
		this.angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, CAN_TIMEOUT_MS);
		this.angleMotor.setSensorPhase(true);
		this.angleMotor.config_kP(0, 30, CAN_TIMEOUT_MS);
		this.angleMotor.config_kI(0, 0.001, CAN_TIMEOUT_MS);
		this.angleMotor.config_kD(0, 200, CAN_TIMEOUT_MS);
		this.angleMotor.config_kF(0, 0, CAN_TIMEOUT_MS);
		this.angleMotor.setNeutralMode(NeutralMode.Brake);

		// Configure drive motor
		this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, CAN_TIMEOUT_MS);
		this.driveMotor.setSensorPhase(true);
		this.driveMotor.config_kP(0, 15, CAN_TIMEOUT_MS);
		this.driveMotor.config_kI(0, 0.01, CAN_TIMEOUT_MS);
		this.driveMotor.config_kD(0, 0.1, CAN_TIMEOUT_MS);
		this.driveMotor.config_kF(0, 0.2, CAN_TIMEOUT_MS);

		// Setup current limiting
		this.driveMotor.configContinuousCurrentLimit(30, CAN_TIMEOUT_MS);
		this.driveMotor.configPeakCurrentLimit(30, CAN_TIMEOUT_MS);
		this.driveMotor.configPeakCurrentDuration(100, CAN_TIMEOUT_MS);
		this.driveMotor.enableCurrentLimit(true);
	}

	@Override
	protected double getAngleEncoderRotations() {
		return angleMotor.getSelectedSensorPosition(0) * (1.0 / 1024.0);
	}

	@Override
	public double getCurrentDistance() {
		return driveMotor.getSelectedSensorPosition(0) * (1.0 / TICKS_PER_INCH) * (driveMotor.getInverted() ? -1 : 1);
	}

	@Override
	public double getCurrentRate() {
		return driveMotor.getSelectedSensorVelocity(0) * (1.0 / TICKS_PER_INCH) * (100.0 / Constants.MILLISECONDS);
	}

	@Override
	protected void setTargetAngleRotations(double rotations) {
		angleMotor.set(ControlMode.Position, rotations * (1024.0 / 1.0));
	}

	@Override
	protected void setDriveMotorInverted(boolean inverted) {
		driveMotor.setInverted(inverted);
	}

	@Override
	public double getCurrentDrivePercentage() {
		return driveMotor.getMotorOutputPercent();
	}

	@Override
	public void setTargetDrivePercentage(double percentage) {
		if (isInverted()) {
			percentage = -percentage;
		}

		driveMotor.set(ControlMode.PercentOutput, percentage);
	}

	@Override
	public double getWheelScrubFactor() {
		return SCRUB_FACTOR;
	}

	@Override
	public void zeroDistance() {
		driveMotor.setSelectedSensorPosition(0, 0, CAN_TIMEOUT_MS);
	}
}
