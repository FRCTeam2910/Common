package org.frcteam2910.common.robot.subsystems;

import org.frcteam2910.common.math.Vector2;

public abstract class HolonomicDrivetrain extends Drivetrain {

    public final void holonomicDrive(Vector2 translation, double rotation) {
        holonomicDrive(translation, rotation, false);
    }

    public abstract void holonomicDrive(Vector2 translation, double rotation, boolean fieldOriented);
}
