package org.frcteam2910.common.robot.input;

import edu.wpi.first.wpilibj.buttons.Button;

public final class NullButton extends Button {
	private boolean value;

	public NullButton() {
		this(false);
	}

	public NullButton(boolean initialValue) {
		value = initialValue;
	}

	@Override
	public boolean get() {
		return value;
	}

	public void set(boolean value) {
		this.value = value;
	}
}
