package org.frcteam2910.common.kinematics;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Twist2;
import org.frcteam2910.common.math.Vector2;

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

    public void resetPose(RigidTransform2 pose) {
        this.pose = pose;
    }

    public void resetPose(Vector2 position, Rotation2 gyroAngle) {
        resetPose(new RigidTransform2(position, gyroAngle));
    }

    public void resetRotation(Rotation2 gyroAngle) {
        resetPose(new RigidTransform2(pose.translation, gyroAngle));
    }

    public RigidTransform2 getPose() {
        return pose;
    }

    public RigidTransform2 update(Rotation2 gyroAngle, double dt, Vector2... moduleVelocities) {
        ChassisVelocity velocity = kinematics.toChassisVelocity(moduleVelocities);

        RigidTransform2 newPose = pose.exp(
                new Twist2(
                        velocity.getTranslationalVelocity().x * dt,
                        velocity.getTranslationalVelocity().y * dt,
                        gyroAngle.rotateBy(pose.rotation.inverse()).toRadians()
                )
        );

        pose = new RigidTransform2(newPose.translation, gyroAngle);

        return pose;
    }
}
