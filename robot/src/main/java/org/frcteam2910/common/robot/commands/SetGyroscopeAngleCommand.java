package org.frcteam2910.common.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.math.Rotation2;

public class SetGyroscopeAngleCommand extends InstantCommand {
    public SetGyroscopeAngleCommand(Gyroscope gyroscope, double angle) {
        super(() -> {
            gyroscope.setAdjustmentAngle(
                    gyroscope.getUnadjustedAngle().rotateBy(Rotation2.fromRadians(angle))
            );
        });
    }
}
