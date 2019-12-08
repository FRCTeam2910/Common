package frc.swerverobot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.swerverobot.subsystems.DrivetrainSubsystem;
import frc.swerverobot.Robot;
import org.frcteam2910.common.math.Vector2;

public class DriveCommand extends Command {
    public DriveCommand() {
        requires(DrivetrainSubsystem.getInstance());
    }

    @Override
    protected void execute() {
        double forward = Robot.getOi().getDriveForwardAxis().get(true);
        double strafe = Robot.getOi().getDriveStrafeAxis().get(true);
        double rotation = Robot.getOi().getDriveRotationAxis().get(true);

        DrivetrainSubsystem.getInstance().drive(new Vector2(forward, strafe), rotation, true);
    }

    @Override
    protected void end() {
        DrivetrainSubsystem.getInstance().drive(Vector2.ZERO, 0.0, false);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
