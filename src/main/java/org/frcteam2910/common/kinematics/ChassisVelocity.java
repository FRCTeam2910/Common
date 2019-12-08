package org.frcteam2910.common.kinematics;

import org.frcteam2910.common.math.Vector2;

/**
 * Represents the velocity of a robot chassis w.r.t. the robot frame of reference.
 * <p>
 * In non-holonomic drive bases, the {@code y} component of the translational velocity should always be 0 because
 * they cannot move sideways.
 */
public class ChassisVelocity {
    private final Vector2 translationalVelocity;
    private final double angularVelocity;

    public ChassisVelocity(Vector2 translationalVelocity, double angularVelocity) {
        this.translationalVelocity = translationalVelocity;
        this.angularVelocity = angularVelocity;
    }

    /**
     * Gets the translational velocity component of this chassis velocity.
     * <p>
     * The {@code x} component of the translational velocity is the robot's forward velocity (positive is forwards) and
     * the {@code y} component is the sideways velocity (positive is to the left).
     *
     * @return The translational velocity w.r.t. the robot.
     */
    public Vector2 getTranslationalVelocity() {
        return translationalVelocity;
    }

    /**
     * Gets the angular velocity component of this chassis velocity.
     * <p>
     * Positive velocities are counter-clockwise movement.
     *
     * @return The angular velocity.
     */
    public double getAngularVelocity() {
        return angularVelocity;
    }
}
