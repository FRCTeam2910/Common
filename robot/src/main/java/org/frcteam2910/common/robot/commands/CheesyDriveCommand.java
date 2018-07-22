package org.frcteam2910.common.robot.commands;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.NullButton;
import org.frcteam2910.common.robot.subsystems.ShiftingTankDrivetrain;
import org.frcteam2910.common.robot.subsystems.TankDrivetrain;

public final class CheesyDriveCommand extends Command {
	private static final double HIGH_WHEEL_NON_LINEARITY = 0.65;
	private static final double LOW_WHEEL_NON_LINEARITY = 0.5;

	private static final double HIGH_NEG_INERTIA_SCALAR = 4.0;

	private static final double LOW_NEG_INERTIA_THRESHOLD = 0.65;
	private static final double LOW_NEG_INERTIA_TURN_SCALAR = 3.5;
	private static final double LOW_NEG_INERTIA_CLOSE_SCALAR = 4.0;
	private static final double LOW_NEG_INERTIA_FAR_SCALAR = 5.0;

	private static final double HIGH_SENSITIVITY = 0.95;
	private static final double LOW_SENSITIVITY = 1.3;

	private static final double QUICK_STOP_DEADBAND = 0.2;
	private static final double QUICK_STOP_WEIGHT = 0.1;
	private static final double QUICK_STOP_SCALAR = 5.0;

	private final TankDrivetrain drivetrain;

	private final Axis forwardAxis;
	private final Axis turnAxis;
	private final Button quickTurnButton;

	private double oldTurn;
	private double quickStopAccumulator;
	private double negInertiaAccumulator;

	public CheesyDriveCommand(TankDrivetrain drivetrain, Axis forwardAxis, Axis turnAxis) {
		this(drivetrain, forwardAxis, turnAxis, new NullButton(false));
	}

	public CheesyDriveCommand(TankDrivetrain drivetrain, Axis forwardAxis, Axis turnAxis, Button quickTurnButton) {
		this.drivetrain = drivetrain;
		this.forwardAxis = forwardAxis;
		this.turnAxis = turnAxis;
		this.quickTurnButton = quickTurnButton;

		requires(drivetrain);
	}

	@Override
	protected void initialize() {
		oldTurn = 0;
		quickStopAccumulator = 0;
		negInertiaAccumulator = 0;
	}

	@Override
	protected void execute() {
		double throttle = forwardAxis.get(true);
		double wheel = turnAxis.get(true);

		boolean quickTurn = quickTurnButton.get();

		double negInertia = wheel - oldTurn;
		oldTurn = wheel;

		double wheelNonLinearity;
		if (!(drivetrain instanceof ShiftingTankDrivetrain) || ((ShiftingTankDrivetrain) drivetrain).inHighGear()) {
			wheelNonLinearity = HIGH_WHEEL_NON_LINEARITY;
			final double denominator = Math.sin(Math.PI / 2 * wheelNonLinearity);

			wheel = Math.sin(Math.PI / 2 * wheelNonLinearity * wheel) / denominator;
			wheel = Math.sin(Math.PI / 2 * wheelNonLinearity * wheel) / denominator;
		} else {
			wheelNonLinearity = LOW_WHEEL_NON_LINEARITY;
			final double denominator = Math.sin(Math.PI / 2 * wheelNonLinearity);

			wheel = Math.sin(Math.PI / 2 * wheelNonLinearity * wheel) / denominator;
			wheel = Math.sin(Math.PI / 2 * wheelNonLinearity * wheel) / denominator;
			wheel = Math.sin(Math.PI / 2 * wheelNonLinearity * wheel) / denominator;
		}
		double left, right, overPower;
		double sensitivity;

		double angularPower;
		double linearPower;

		double negInertiaScalar;
		if (!(drivetrain instanceof ShiftingTankDrivetrain) || ((ShiftingTankDrivetrain) drivetrain).inHighGear()) {
			negInertiaScalar = HIGH_NEG_INERTIA_SCALAR;
			sensitivity = HIGH_SENSITIVITY;
		} else {
			if (wheel * negInertia > 0) {
				negInertiaScalar = LOW_NEG_INERTIA_TURN_SCALAR;
			} else {
				if (Math.abs(wheel) > LOW_NEG_INERTIA_THRESHOLD) {
					negInertiaScalar = LOW_NEG_INERTIA_FAR_SCALAR;
				} else {
					negInertiaScalar = LOW_NEG_INERTIA_CLOSE_SCALAR;
				}
			}
			sensitivity = LOW_SENSITIVITY;
		}

		double negInertiaPower = negInertia * negInertiaScalar;
		negInertiaAccumulator += negInertiaPower;

		wheel = wheel + negInertiaAccumulator;
		if (negInertiaAccumulator > 1)
			negInertiaAccumulator -= 1;
		else if (negInertiaAccumulator < -1)
			negInertiaAccumulator += 1;
		else
			negInertiaAccumulator = 0;
		linearPower = throttle;

		if (quickTurn) {
			if (Math.abs(linearPower) < QUICK_STOP_DEADBAND) {
				double alpha = QUICK_STOP_WEIGHT;
				quickStopAccumulator = (1 - alpha) * quickStopAccumulator + alpha * MathUtils.clamp(wheel, -1, 1) * QUICK_STOP_SCALAR;
			}
			overPower = 1;
			angularPower = wheel;
		} else {
			overPower = 0;
			angularPower = Math.abs(throttle) * wheel * sensitivity - quickStopAccumulator;
			if (quickStopAccumulator > 1)
				quickStopAccumulator -= 1;
			else if (quickStopAccumulator < -1)
				quickStopAccumulator += 1;
			else
				quickStopAccumulator = 0;
		}

		right = left = linearPower;
		left += angularPower;
		right -= angularPower;

		if (left > 1) {
			right -= overPower * (left - 1);
			left = 1;
		} else if (right > 1) {
			left -= overPower * (right - 1);
			right = 1;
		} else if (left < -1) {
			right += overPower * (-1 - left);
			left = -1;
		} else if (right < -1) {
			left += overPower * (-1 - right);
			right = -1;
		}

		drivetrain.tankDrive(left, right);
	}

	@Override
	protected void end() {
		drivetrain.stop();
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
