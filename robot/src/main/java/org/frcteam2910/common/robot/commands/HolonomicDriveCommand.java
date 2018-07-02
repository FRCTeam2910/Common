package org.frcteam2910.common.robot.commands;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.subsystems.HolonomicDrivetrain;

public final class HolonomicDriveCommand extends Command {
	private final HolonomicDrivetrain drivetrain;

	private final Axis forwardAxis;
	private final Axis strafeAxis;
	private final Axis rotationAxis;
	private final Button fieldOrientedOverrideButton;

	public HolonomicDriveCommand(HolonomicDrivetrain drivetrain, Axis forwardAxis, Axis strafeAxis, Axis rotationAxis, Button fieldOrientedOverrideButton) {
		this.drivetrain = drivetrain;
		this.forwardAxis = forwardAxis;
		this.strafeAxis = strafeAxis;
		this.rotationAxis = rotationAxis;
		this.fieldOrientedOverrideButton = fieldOrientedOverrideButton;

		requires(drivetrain);
	}

	@Override
	protected void execute() {
		drivetrain.holonomicDrive(
				new Vector2(forwardAxis.get(true), strafeAxis.get(true)),
				rotationAxis.get(true),
				!fieldOrientedOverrideButton.get()
		);
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
