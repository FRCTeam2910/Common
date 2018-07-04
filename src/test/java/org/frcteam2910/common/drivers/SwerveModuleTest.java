package org.frcteam2910.common.drivers;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class SwerveModuleTest {
	@Test
	public void setTargetAngleTest() {
		MockSwerveModule module = new MockSwerveModule(Vector2.ZERO, Rotation2.ZERO);

		module.setAngleRotations(0.0);
		module.setTargetAngle(Rotation2.fromDegrees(0.0));
		assertEquals(0.0, module.getTargetAngleRotations(), MathUtils.EPSILON);
		assertFalse(module.isDriveInverted());

		module.setAngleRotations(0.5);
		module.setTargetAngle(Rotation2.fromDegrees(0.0));
		assertEquals(0.5, module.getTargetAngleRotations(), MathUtils.EPSILON);
		assertTrue(module.isDriveInverted());

		module.setAdjustmentAngle(Rotation2.fromDegrees(180.0));
		module.setAngleRotations(0.5);
		module.setTargetAngle(Rotation2.fromDegrees(0.0));
		assertEquals(0.5, module.getTargetAngleRotations(), MathUtils.EPSILON);
		assertFalse(module.isDriveInverted());

		module.setAngleRotations(0.6);
		module.setTargetAngle(Rotation2.fromDegrees(180.0));
		assertEquals(0.5, module.getTargetAngleRotations(), MathUtils.EPSILON);
		assertTrue(module.isDriveInverted());
	}

	@Test
	public void kinematicsTest() {
		MockSwerveModule module = new MockSwerveModule(Vector2.ZERO, Rotation2.ZERO);
		module.setCurrentDistance(0.0);
		module.updateKinematics(Rotation2.ZERO);
		module.setCurrentDistance(1);
		module.updateKinematics(Rotation2.ZERO);

		assertEquals(new Vector2(1, 0.0), module.getKinematicPosition());


		module = new MockSwerveModule(Vector2.ZERO, Rotation2.ZERO);

		module.setCurrentDistance(0.0);
		module.setAngleRotations(0.25);
		module.updateKinematics(Rotation2.fromDegrees(-90.0));
		module.setCurrentDistance(5.0);
		module.updateKinematics(Rotation2.fromDegrees(-90.0));

		assertEquals(new Vector2(5, 0), module.getKinematicPosition());
	}
}
