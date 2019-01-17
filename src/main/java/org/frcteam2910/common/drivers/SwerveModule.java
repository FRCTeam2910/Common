package org.frcteam2910.common.drivers;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public abstract class SwerveModule {
    private final Vector2 modulePosition;

    private AtomicLong currentAngle = new AtomicLong();
    private AtomicLong currentDistance = new AtomicLong();

    private AtomicReference<Vector2> targetVelocity = new AtomicReference<>(Vector2.ZERO);

    private AtomicReference<Vector2> currentPosition = new AtomicReference<>(Vector2.ZERO);

    private double previousDistance;
    private String name = "Unknown";

    public SwerveModule(Vector2 modulePosition) {
        this.modulePosition = modulePosition;
    }

    /**
     * Sets the module's name.
     *
     * @param name the module's name
     */
    public void setName(String name) {
        this.name = name;
    }

    /**
     * Gets the module's name.
     *
     * @return the module's name
     */
    public String getName() {
        return name;
    }

    /**
     * Reads the angle encoder to get the angle of the module.
     * <p>
     * The angle returned is in radians and MUST be inside the range [0, 2&pi;)
     *
     * @return the angle of the module in radians
     */
    protected abstract double readAngle();

    /**
     * Reads the drive encoder to get the displacement of the module wheel in inches.
     *
     * @return the displacement of the module in inches
     */
    protected abstract double readDistance();

    /**
     * Sets the target angle.
     *
     * @param angle the target angle in radians
     */
    protected abstract void setTargetAngle(double angle);

    /**
     * Sets the output of the drive motor.
     *
     * @param output the output that the drive motor should be set to
     */
    protected abstract void setDriveOutput(double output);

    /**
     * Gets the distance the module is from the robot's center of rotation (usually the center).
     *
     * @return The distance the module is from the robot's center of rotation.
     */
    public final Vector2 getModulePosition() {
        return modulePosition;
    }

    /**
     * Gets the current angle of the module in radians.
     *
     * @return The current angle of the module.
     */
    public final double getCurrentAngle() {
        return Double.longBitsToDouble(currentAngle.get());
    }

    /**
     * Gets the distance the module has driven since the robot started up.
     *
     * @return the distance driven
     */
    public final double getCurrentDistance() {
        return Double.longBitsToDouble(currentDistance.get());
    }

    /**
     * Sets the target velocity. The vector should have a length that is less than or equal to 1.
     *
     * @param velocity the target velocity
     */
    public final void setTargetVelocity(Vector2 velocity) {
        targetVelocity.set(velocity);
    }

    /**
     * Gets the current position.
     *
     * @return the current position
     */
    public final Vector2 getCurrentPosition() {
        return currentPosition.get();
    }

    /**
     * Resets the module's position to (0, 0)
     */
    public void resetKinematics() {
        resetKinematics(Vector2.ZERO);
    }

    /**
     * Resets the module's position to the desired position.
     *
     * @param position the position to reset to
     */
    public void resetKinematics(Vector2 position) {
        currentPosition.set(position);
    }

    /**
     * Updates the sensor readings that the module uses.
     */
    public void updateSensors() {
        currentAngle.set(Double.doubleToRawLongBits(readAngle()));
        currentDistance.set(Double.doubleToRawLongBits(readDistance()));
    }

    /**
     * Updates the module's position.
     *
     * @param robotRotation the rotation of the robot in radians
     */
    public void updateKinematics(double robotRotation) {
        double currentDistance = getCurrentDistance();
        double deltaDistance = currentDistance - previousDistance;
        double currentAngle = getCurrentAngle() + robotRotation;

        Vector2 deltaPosition = Vector2.fromAngle(Rotation2.fromRadians(currentAngle)).scale(deltaDistance);

        currentPosition.updateAndGet(current -> current.add(deltaPosition));
        previousDistance = currentDistance;
    }

    /**
     * Uses the target velocity to update the module's outputs.
     *
     * @param dt update loop delta time
     */
    public void updateState(double dt) {
        Vector2 targetVelocity = this.targetVelocity.get();

        double targetAngle = targetVelocity.getAngle().toRadians();
        double targetDriveOutput = targetVelocity.length;

        final double currentAngle = getCurrentAngle();

        // Change the target angle so the delta is in the range [-pi, pi) instead of [0, 2pi)
        double delta = targetAngle - currentAngle;
        if (delta >= Math.PI) {
            targetAngle -= 2.0 * Math.PI;
        } else if (delta < -Math.PI) {
            targetAngle += 2.0 * Math.PI;
        }

        // Deltas that are greater than 90 deg or less than -90 deg can be inverted so the total movement of the module
        // is less than 90 deg by inverting the wheel direction
        delta = targetAngle - currentAngle;
        if (delta > Math.PI / 2.0 || delta < -Math.PI / 2.0) {
            // Only need to add pi here because the target angle will be put back into the range [0, 2pi)
            targetAngle += Math.PI;

            targetDriveOutput *= -1.0;
        }

        // Put target angle back into the range [0, 2pi)
        targetAngle %= 2.0 * Math.PI;
        if (targetAngle < 0.0) {
            targetAngle += 2.0 * Math.PI;
        }

        setTargetAngle(targetAngle);
        setDriveOutput(targetDriveOutput);
    }
}
