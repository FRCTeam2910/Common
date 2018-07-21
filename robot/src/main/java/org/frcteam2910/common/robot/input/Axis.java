package org.frcteam2910.common.robot.input;

import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.common.robot.Utilities;

public abstract class Axis {
	public static final double DEADBAND = 0.025;

	private boolean inverted = false;
	private double scale = 1.0;

	public boolean isInverted() {
		return inverted;
	}

	public void setInverted(boolean inverted) {
		this.inverted = inverted;
	}

	public double getScale() {
		return scale;
	}

	public void setScale(double scale) {
		this.scale = scale;
	}

	public abstract double getRaw();

	public double get() {
		return get(false, false);
	}

	public double get(boolean squared) {
		return get(squared, false);
	}

	public double get(boolean squared, boolean ignoreScale) {
		double value = getRaw();

		// Invert if axis is inverted
		if (inverted) {
			value = -value;
		}

		// Deadband value
		value = Utilities.deadband(value, DEADBAND);

		// Square value
		if (squared) {
			value = Math.copySign(value * value, value);
		}

		// Scale value
		if (!ignoreScale) {
			value *= scale;
		}

		return value;
	}

	public abstract void whenPressed(final Command command);

	public abstract void whileHeld(final Command command);

	public abstract void whenReleased(final Command command);

	public abstract void toggleWhenPressed(final Command command);

	public abstract void cancelWhenPressed(final Command command);
}
