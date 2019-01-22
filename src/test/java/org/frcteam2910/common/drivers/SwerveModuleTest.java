package org.frcteam2910.common.drivers;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class SwerveModuleTest {
    private static final double UPDATE_DT = 5.0e-3;

    @Test
    public void setTargetVelocityTest() {
        MockSwerveModule module = new MockSwerveModule(Vector2.ZERO);

        module.writeAngle(0.0);
        module.setTargetVelocity(new Vector2(1.0, 0.0));
        module.updateSensors();
        module.updateState(UPDATE_DT);
        assertEquals(0.0, module.getTargetAngle(), MathUtils.EPSILON);
        assertEquals(1.0, module.getDriveOutput(), MathUtils.EPSILON);

        module.writeAngle(Math.PI);
        module.setTargetVelocity(new Vector2(1.0, 0.0));
        module.updateSensors();
        module.updateState(UPDATE_DT);
        assertEquals(Math.PI, module.getTargetAngle(), MathUtils.EPSILON);
        assertEquals(-1.0, module.getDriveOutput(), MathUtils.EPSILON);

        module.writeAngle(Math.PI);
        module.setTargetVelocity(new Vector2(-1.0, 1.0).normal());
        module.updateSensors();
        module.updateState(UPDATE_DT);
        assertEquals(0.75 * Math.PI, module.getTargetAngle(), MathUtils.EPSILON);
        assertEquals(1.0, module.getDriveOutput(), MathUtils.EPSILON);

        module.writeAngle(1.2 * Math.PI);
        module.setTargetVelocity(new Vector2(-1.0, 0.0));
        module.updateSensors();
        module.updateState(UPDATE_DT);
        assertEquals(Math.PI, module.getTargetAngle(), MathUtils.EPSILON);
        assertEquals(1.0, module.getDriveOutput(), MathUtils.EPSILON);

        module.writeAngle(Math.toRadians(5.0));
        module.setTargetVelocity(Vector2.fromAngle(Rotation2.fromDegrees(-5.0)));
        module.updateSensors();
        module.updateState(UPDATE_DT);
        assertEquals(Math.toRadians(360.0 - 5.0), module.getTargetAngle(), MathUtils.EPSILON);

        module.writeAngle(Math.toRadians(360.0 - 5.0));
        module.setTargetVelocity(Vector2.fromAngle(Rotation2.fromDegrees(5.0)));
        module.updateSensors();
        module.updateState(UPDATE_DT);
        assertEquals(Math.toRadians(5.0), module.getTargetAngle(), MathUtils.EPSILON);

        module.writeAngle(Math.toRadians(175.0));
        module.setTargetVelocity(Vector2.fromAngle(Rotation2.fromDegrees(185.0)));
        module.updateSensors();
        module.updateState(UPDATE_DT);
        assertEquals(Math.toRadians(185.0), module.getTargetAngle(), MathUtils.EPSILON);
    }

    @Test
    public void setNameTest() {
        SwerveModule module = new MockSwerveModule(Vector2.ZERO);
        assertEquals("Initial module name is not correct", "Unknown", module.getName());

        module.setName("Test");

        assertEquals("Name did not update", "Test", module.getName());
    }

    @Test
    public void kinematicsTest() {
        MockSwerveModule module = new MockSwerveModule(Vector2.ZERO);
        module.writeDistance(0.0);
        module.updateSensors();
        module.updateKinematics(0.0);
        module.writeDistance(1.0);
        module.updateSensors();
        module.updateKinematics(0.0);

        assertEquals(new Vector2(1.0, 0.0), module.getCurrentPosition());


        module = new MockSwerveModule(Vector2.ZERO);

        module.writeDistance(0.0);
        module.writeAngle(0.5 * Math.PI);
        module.updateSensors();
        module.updateKinematics(1.5 * Math.PI);
        module.writeDistance(5.0);
        module.updateSensors();
        module.updateKinematics(1.5 * Math.PI);

        assertEquals(new Vector2(5, 0), module.getCurrentPosition());
    }
}
