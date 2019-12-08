package org.frcteam2910.common.kinematics;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class SwerveKinematicsTest {
    private final Vector2 frontLeft = new Vector2(12.0, 12.0);
    private final Vector2 frontRight = new Vector2(12.0, -12.0);
    private final Vector2 backLeft = new Vector2(-12.0, 12.0);
    private final Vector2 backRight = new Vector2(-12.0, -12.0);

    private final SwerveKinematics kinematics = new SwerveKinematics(frontLeft, frontRight, backLeft, backRight);

    @Test
    public void straightLineInverseKinematicsTest() {
        ChassisVelocity velocity = new ChassisVelocity(new Vector2(5.0, 0.0), 0.0);
        var moduleVelocities = kinematics.toModuleVelocities(velocity);

        assertEquals(5.0, moduleVelocities[0].length, MathUtils.EPSILON);
        assertEquals(5.0, moduleVelocities[1].length, MathUtils.EPSILON);
        assertEquals(5.0, moduleVelocities[2].length, MathUtils.EPSILON);
        assertEquals(5.0, moduleVelocities[3].length, MathUtils.EPSILON);
        assertEquals(0.0, moduleVelocities[0].getAngle().toDegrees(), MathUtils.EPSILON);
        assertEquals(0.0, moduleVelocities[1].getAngle().toDegrees(), MathUtils.EPSILON);
        assertEquals(0.0, moduleVelocities[2].getAngle().toDegrees(), MathUtils.EPSILON);
        assertEquals(0.0, moduleVelocities[3].getAngle().toDegrees(), MathUtils.EPSILON);
    }

    @Test
    public void straightLineForwardKinematicsTest() {
        Vector2 moduleVelocity = new Vector2(5.0, 0.0);
        var velocity = kinematics.toChassisVelocity(moduleVelocity, moduleVelocity, moduleVelocity, moduleVelocity);

        assertEquals(5.0, velocity.getTranslationalVelocity().x, MathUtils.EPSILON);
        assertEquals(0.0, velocity.getTranslationalVelocity().y, MathUtils.EPSILON);
        assertEquals(0.0, velocity.getAngularVelocity(), MathUtils.EPSILON);
    }

    @Test
    public void straightStrafeInverseKinematicsTest() {
        ChassisVelocity velocity = new ChassisVelocity(new Vector2(0.0, 5.0), 0.0);
        var moduleVelocities = kinematics.toModuleVelocities(velocity);

        assertEquals(5.0, moduleVelocities[0].length, MathUtils.EPSILON);
        assertEquals(5.0, moduleVelocities[1].length, MathUtils.EPSILON);
        assertEquals(5.0, moduleVelocities[2].length, MathUtils.EPSILON);
        assertEquals(5.0, moduleVelocities[3].length, MathUtils.EPSILON);
        assertEquals(90.0, moduleVelocities[0].getAngle().toDegrees(), MathUtils.EPSILON);
        assertEquals(90.0, moduleVelocities[1].getAngle().toDegrees(), MathUtils.EPSILON);
        assertEquals(90.0, moduleVelocities[2].getAngle().toDegrees(), MathUtils.EPSILON);
        assertEquals(90.0, moduleVelocities[3].getAngle().toDegrees(), MathUtils.EPSILON);
    }

    @Test
    public void straightStrafeForwardKinematicsTest() {
        Vector2 moduleVelocity = new Vector2(0.0, 5.0);
        var velocity = kinematics.toChassisVelocity(moduleVelocity, moduleVelocity, moduleVelocity, moduleVelocity);

        assertEquals(0.0, velocity.getTranslationalVelocity().x, MathUtils.EPSILON);
        assertEquals(5.0, velocity.getTranslationalVelocity().y, MathUtils.EPSILON);
        assertEquals(0.0, velocity.getAngularVelocity(), MathUtils.EPSILON);
    }

    @Test
    public void turnInPlaceInverseKinematicsTest() {
        ChassisVelocity velocity = new ChassisVelocity(Vector2.ZERO, 2.0 * Math.PI);
        var moduleVelocities = kinematics.toModuleVelocities(velocity);

        assertEquals(106.63, moduleVelocities[0].length, 0.1);
        assertEquals(106.63, moduleVelocities[1].length, 0.1);
        assertEquals(106.63, moduleVelocities[2].length, 0.1);
        assertEquals(106.63, moduleVelocities[3].length, 0.1);
        assertEquals(135.0, moduleVelocities[0].getAngle().toDegrees(), MathUtils.EPSILON);
        assertEquals(45.0, moduleVelocities[1].getAngle().toDegrees(), MathUtils.EPSILON);
        assertEquals(225.0, moduleVelocities[2].getAngle().toDegrees(), MathUtils.EPSILON);
        assertEquals(315.0, moduleVelocities[3].getAngle().toDegrees(), MathUtils.EPSILON);
    }

    @Test
    public void turnInPlaceForwardKinematicsTest() {
        Vector2 frontLeftVelocity = Vector2.fromAngle(Rotation2.fromDegrees(135.0)).scale(106.629);
        Vector2 frontRightVelocity = Vector2.fromAngle(Rotation2.fromDegrees(45.0)).scale(106.629);
        Vector2 backLeftVelocity = Vector2.fromAngle(Rotation2.fromDegrees(225.0)).scale(106.629);
        Vector2 backRightVelocity = Vector2.fromAngle(Rotation2.fromDegrees(315.0)).scale(106.629);

        var velocity = kinematics.toChassisVelocity(frontLeftVelocity, frontRightVelocity, backLeftVelocity, backRightVelocity);

        assertEquals(0.0, velocity.getTranslationalVelocity().x, MathUtils.EPSILON);
        assertEquals(0.0, velocity.getTranslationalVelocity().y, MathUtils.EPSILON);
        assertEquals(2.0 * Math.PI, velocity.getAngularVelocity(), 0.1);
    }

    @Test
    public void normalizeModuleVelocitiesTest() {
        Vector2 fl = new Vector2(5, 0.0);
        Vector2 fr = new Vector2(6, 0.0);
        Vector2 bl = new Vector2(4, 0.0);
        Vector2 br = new Vector2(7, 0.0);

        Vector2[] moduleVelocities = {fl, fr, bl, br};
        SwerveKinematics.normalizeModuleVelocities(moduleVelocities, 5.5);

        double factor = 5.5 / 7.0;

        assertEquals(5.0 * factor, moduleVelocities[0].length, MathUtils.EPSILON);
        assertEquals(6.0 * factor, moduleVelocities[1].length, MathUtils.EPSILON);
        assertEquals(4.0 * factor, moduleVelocities[2].length, MathUtils.EPSILON);
        assertEquals(7.0 * factor, moduleVelocities[3].length, MathUtils.EPSILON);
    }

}
