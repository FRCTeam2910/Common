package org.frcteam2910.common.drivers;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public abstract class SwerveModule {
    private final Vector2 modulePosition;

    private final Object sensorMutex = new Object();
    private double currentAngle = 0.0;
    private double currentDistance = 0.0;

    private final Object stateMutex = new Object();
    private double targetSpeed = 0.0;
    private double targetAngle = 0.0;

    private final Object kinematicsMutex = new Object();
    private Vector2 currentPosition = Vector2.ZERO;
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
        synchronized (sensorMutex) {
            return currentAngle;
        }
    }

    /**
     * Gets the distance the module has driven since the robot started up.
     *
     * @return the distance driven
     */
    public final double getCurrentDistance() {
        synchronized (sensorMutex) {
            return currentDistance;
        }
    }

    /**
     * Gets the current velocity of the wheel.
     *
     * TODO: Allow implementors to specify the current velocity without overriding this method.
     * @return the velocity of the module.
     */
    public double getCurrentVelocity() {
        return 0;
    }

    /**
     * Gets the amount of current being drawn by the drive motor.
     *
     * TODO: Allow implementors to specify current draw without overriding this method.
     * @return the amount of current being drawn by the drive motor.
     */
    public double getDriveCurrent() {
        return 0;
    }

    public Vector2 getTargetVelocity() {
        double targetAngle;
        double targetSpeed;

        synchronized (stateMutex) {
            targetAngle = this.targetAngle;
            targetSpeed = this.targetSpeed;
        }

        return Vector2.fromAngle(Rotation2.fromRadians(targetAngle)).scale(targetSpeed);
    }

    /**
     * Sets the target velocity. The vector should have a length that is less than or equal to 1.
     *
     * @param velocity the target velocity
     */
    public final void setTargetVelocity(Vector2 velocity) {
        synchronized (stateMutex) {
            targetSpeed = velocity.length;
            targetAngle = velocity.getAngle().toRadians();
        }
    }

    public final void setTargetVelocity(double speed, double angle) {
        if (speed < 0.0) {
            speed *= -1.0;

            angle += Math.PI;
        }

        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        synchronized (stateMutex) {
            targetSpeed = speed;
            targetAngle = angle;
        }
    }

    /**
     * Gets the current position.
     *
     * @return the current position
     */
    public final Vector2 getCurrentPosition() {
        synchronized (kinematicsMutex) {
            return currentPosition;
        }
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
        synchronized (kinematicsMutex) {
            currentPosition = position;
        }
    }

    /**
     * Updates the sensor readings that the module uses.
     */
    public void updateSensors() {
        synchronized (sensorMutex) {
            currentAngle = readAngle();
            currentDistance = readDistance();
        }
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

        synchronized (kinematicsMutex) {
            currentPosition = currentPosition.add(deltaPosition);
            previousDistance = currentDistance;
        }
    }

    /**
     * Uses the target velocity to update the module's outputs.
     *
     * @param dt update loop delta time
     */
    public void updateState(double dt) {
        double targetAngle;
        double targetSpeed;

        synchronized (stateMutex) {
            targetAngle = this.targetAngle;
            targetSpeed = this.targetSpeed;
        }

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

            targetSpeed *= -1.0;
        }

        // Put target angle back into the range [0, 2pi)
        targetAngle %= 2.0 * Math.PI;
        if (targetAngle < 0.0) {
            targetAngle += 2.0 * Math.PI;
        }

        setTargetAngle(targetAngle);
        setDriveOutput(targetSpeed);
    }
}
