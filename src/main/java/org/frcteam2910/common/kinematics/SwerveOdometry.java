package org.frcteam2910.common.kinematics;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

/**
 * Helper class for swerve drive odometry.
 * <p>
 * Odometry allows a robot to track what it's position on the field is using the encoders on it's swerve modules.
 */
public class SwerveOdometry {
    private final SwerveKinematics kinematics;
    private RigidTransform2 pose;

    public SwerveOdometry(SwerveKinematics kinematics) {
        this(kinematics, RigidTransform2.ZERO);
    }

    public SwerveOdometry(SwerveKinematics kinematics, RigidTransform2 initialPose) {
        this.kinematics = kinematics;
        this.pose = initialPose;
    }

    /**
     * Resets the robot's pose.
     *
     * @param pose The robot's new pose.
     */
    public void resetPose(RigidTransform2 pose) {
        this.pose = pose;
    }

    /**
     * Resets the robot's pose.
     *
     * @param position The new position of the robot.
     * @param rotation The new rotation of the robot.
     */
    public void resetPose(Vector2 position, Rotation2 rotation) {
        resetPose(new RigidTransform2(position, rotation));
    }

    /**
     * Resets the robot's position.
     *
     * @param position The robot's new position.
     */
    public void resetPosition(Vector2 position) {
        resetPose(position, getPose().rotation);
    }

    /**
     * Resets the robot's rotation.
     * <p>
     * This should be called when the gyroscope's angle is reset.
     *
     * @param rotation The robot's new rotation.
     */
    public void resetRotation(Rotation2 rotation) {
        resetPose(getPose().translation, rotation);
    }

    /**
     * Gets the position of the robot.
     *
     * @return The pose of the robot.
     */
    public RigidTransform2 getPose() {
        return pose;
    }

    /**
     * Updates the robot's position using forward kinematics and integration of the pose over time.
     *
     * @param gyroAngle        The angle from the gyroscope.
     * @param dt               The change in time.
     * @param moduleVelocities The velocities of the swerve modules. The modules must be in the same order that
     *                         {@link SwerveKinematics} was given when it was instantiated.
     * @return The new pose of the robot.
     */
    public RigidTransform2 update(Rotation2 gyroAngle, double dt, Vector2... moduleVelocities) {
        ChassisVelocity velocity = kinematics.toChassisVelocity(moduleVelocities);

        // Calculate the field-oriented translational velocity of the robot
        Vector2 fieldOrientedVelocity = velocity.getTranslationalVelocity().rotateBy(gyroAngle);

        // Integrate using dt to determine our new position
        Vector2 newPosition = pose.translation
                .add(fieldOrientedVelocity.scale(dt));

        pose = new RigidTransform2(newPosition, gyroAngle);

        return pose;
    }
}
