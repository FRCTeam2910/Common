package org.frcteam2910.common.kinematics;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class SwerveOdometryTest {
    private final Vector2 frontLeft = new Vector2(12.0, 12.0);
    private final Vector2 frontRight = new Vector2(12.0, -12.0);
    private final Vector2 backLeft = new Vector2(-12.0, 12.0);
    private final Vector2 backRight = new Vector2(-12.0, -12.0);

    private final SwerveKinematics kinematics = new SwerveKinematics(frontLeft, frontRight, backLeft, backRight);
    private final SwerveOdometry odometry = new SwerveOdometry(kinematics, RigidTransform2.ZERO);

    @Test
    public void forwardTest() {
        final Vector2[] moduleVelocities = {
                new Vector2(5.0, 0.0),
                new Vector2(5.0, 0.0),
                new Vector2(5.0, 0.0),
                new Vector2(5.0, 0.0)
        };

        odometry.resetPose(RigidTransform2.ZERO);
        var pose = odometry.update(Rotation2.ZERO, 1.0, moduleVelocities);

        assertEquals(5.0, pose.translation.x, 0.01);
        assertEquals(0.0, pose.translation.y, 0.01);
        assertEquals(0.0, pose.rotation.toDegrees(), 0.01);
    }

    @Test
    public void strafeTest() {
        final Vector2[] moduleVelocities = {
                new Vector2(0.0, 5.0),
                new Vector2(0.0, 5.0),
                new Vector2(0.0, 5.0),
                new Vector2(0.0, 5.0)
        };

        odometry.resetPose(RigidTransform2.ZERO);
        var pose = odometry.update(Rotation2.ZERO, 1.0, moduleVelocities);

        assertEquals(0.0, pose.translation.x, 0.01);
        assertEquals(5.0, pose.translation.y, 0.01);
        assertEquals(0.0, pose.rotation.toDegrees(), 0.01);
    }

    @Test
    public void turnInPlaceTest() {
        final Vector2[] moduleVelocities = {
                Vector2.fromAngle(Rotation2.fromDegrees(135.0)).scale(106.629),
                Vector2.fromAngle(Rotation2.fromDegrees(45.0)).scale(106.629),
                Vector2.fromAngle(Rotation2.fromDegrees(225.0)).scale(106.629),
                Vector2.fromAngle(Rotation2.fromDegrees(315.0)).scale(106.629)
        };

        odometry.resetPose(RigidTransform2.ZERO);
        final var pose = odometry.update(Rotation2.fromDegrees(90.0), 1.0, moduleVelocities);

        assertEquals(0.0, pose.translation.x, 0.01);
        assertEquals(0.0, pose.translation.y, 0.01);
        assertEquals(90.0, pose.rotation.toDegrees(), 0.01);
    }

    @Test
    public void gyroAngleResetTest() {
        // Resetting the gyro angle should only change the rotation of the pose
        odometry.resetPose(new RigidTransform2(new Vector2(-254.1323, 1114.2056), Rotation2.fromDegrees(330.0)));
        odometry.resetRotation(Rotation2.fromDegrees(148.2767));
        final var pose = odometry.getPose();

        assertEquals(-254.1323, pose.translation.x, MathUtils.EPSILON);
        assertEquals(1114.2056, pose.translation.y, MathUtils.EPSILON);
        assertEquals(148.2767, pose.rotation.toDegrees(), MathUtils.EPSILON);
    }

    @Test
    public void constructorWithoutInitialPoseIsZeroTest() {
        SwerveOdometry odometry = new SwerveOdometry(kinematics);
        final var pose = odometry.getPose();

        assertEquals(0.0, pose.translation.x, MathUtils.EPSILON);
        assertEquals(0.0, pose.translation.y, MathUtils.EPSILON);
        assertEquals(0.0, pose.rotation.toDegrees(), MathUtils.EPSILON);
    }
}
