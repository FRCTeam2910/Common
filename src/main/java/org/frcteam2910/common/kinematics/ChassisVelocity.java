package org.frcteam2910.common.kinematics;

import org.frcteam2910.common.math.Vector2;

public class ChassisVelocity {
    private final Vector2 translationalVelocity;
    private final double rotationalVelocity;

    public ChassisVelocity(Vector2 translationalVelocity, double rotationalVelocity) {
        this.translationalVelocity = translationalVelocity;
        this.rotationalVelocity = rotationalVelocity;
    }

    public Vector2 getTranslationalVelocity() {
        return translationalVelocity;
    }

    public double getRotationalVelocity() {
        return rotationalVelocity;
    }
}
