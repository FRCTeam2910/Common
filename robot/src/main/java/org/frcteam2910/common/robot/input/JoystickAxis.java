package org.frcteam2910.common.robot.input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.common.math.MathUtils;

public final class JoystickAxis extends Axis {
	private final Joystick joystick;
	private final int axis;
	private final Button button;

	public JoystickAxis(Joystick joystick, int axis) {
		this.joystick = joystick;
		this.axis = axis;

		button = new Button() {
			@Override
			public boolean get() {
				return !MathUtils.epsilonEquals(JoystickAxis.this.get(), 0, 0.1);
			}
		};
	}

	@Override
	public double getRaw() {
		return joystick.getRawAxis(axis);
	}

	@Override
	public void whenPressed(Command command) {
		button.whenPressed(command);
	}

	@Override
	public void whileHeld(Command command) {
		button.whileHeld(command);
	}

	@Override
	public void whenReleased(Command command) {
		button.whenReleased(command);
	}

	@Override
	public void toggleWhenPressed(Command command) {
		button.toggleWhenPressed(command);
	}

	@Override
	public void cancelWhenPressed(Command command) {
		button.cancelWhenPressed(command);
	}
}
