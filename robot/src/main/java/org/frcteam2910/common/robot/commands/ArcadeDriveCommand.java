package org.frcteam2910.common.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.subsystems.TankDrivetrain;

public final class ArcadeDriveCommand extends Command {
	private final TankDrivetrain drivetrain;

	private final Axis forwardAxis;
	private final Axis turnAxis;

	public ArcadeDriveCommand(TankDrivetrain drivetrain, Axis forwardAxis, Axis turnAxis) {
		this.drivetrain = drivetrain;
		this.forwardAxis = forwardAxis;
		this.turnAxis = turnAxis;

		requires(drivetrain);
	}

	@Override
	protected void execute() {
		drivetrain.arcadeDrive(
				forwardAxis.get(true),
				turnAxis.get(true)
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
