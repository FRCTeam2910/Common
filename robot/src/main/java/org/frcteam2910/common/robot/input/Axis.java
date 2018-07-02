package org.frcteam2910.common.robot.input;

import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.common.robot.Utilities;

public abstract class Axis {
	public static final double ZERO_EPSILON = 0.025;

	private boolean inverted = false;

	public boolean isInverted() {
		return inverted;
	}

	public void setInverted(boolean inverted) {
		this.inverted = inverted;
	}

	public abstract double getRaw();

	public double get() {
		return Utilities.deadband(getRaw() * (inverted ? -1 : 1), ZERO_EPSILON);
	}

	public double get(boolean squared) {
		double value = get();
		if (squared)
			value = Math.copySign(value * value, value);
		return value;
	}

	public abstract void whenPressed(final Command command);

	public abstract void whileHeld(final Command command);

	public abstract void whenReleased(final Command command);

	public abstract void toggleWhenPressed(final Command command);

	public abstract void cancelWhenPressed(final Command command);
}
