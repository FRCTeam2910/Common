package org.frcteam2910.common.drivers;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Vector2;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class SwerveModuleTest {
	@Test
	public void setTargetAngleTest() {
		MockSwerveModule module = new MockSwerveModule(new Vector2(0, 0), 0, 4.0);

		module.setAngleRotations(0.0);
		module.setTargetAngle(0.0);
		assertEquals(0.0, module.getTargetAngleRotations(), MathUtils.EPSILON);

		module.setAngleRotations(0.5);
		module.setTargetAngle(0.0);
		assertEquals(0.5, module.getTargetAngleRotations(), MathUtils.EPSILON);

		module.setAdjustmentAngle(180.0);
		module.setAngleRotations(0.5);
		module.setTargetAngle(0.0);
		assertEquals(0.5, module.getTargetAngleRotations(), MathUtils.EPSILON);

		module.setAngleRotations(0.6);
		module.setTargetAngle(180.0);
		assertEquals(0.5, module.getTargetAngleRotations(), MathUtils.EPSILON);
	}

	@Test
	public void kinematicsTest() {
		MockSwerveModule module = new MockSwerveModule(new Vector2(0, 0), 0, 4.0);
		module.setDriveRotations(0.0);
		module.updateKinematics(0.0);
		module.setDriveRotations(1);
		module.updateKinematics(0.0);

		assertEquals(new Vector2(4 * Math.PI, 0.0), module.getKinematicPosition());


		module = new MockSwerveModule(new Vector2(0, 0), 0, 4.0);

		module.setDriveRotations(0.0);
		module.setAngleRotations(0.25);
		module.updateKinematics(-90.0);
		module.setDriveRotations(5.0);
		module.updateKinematics(-90.0);

		assertEquals(new Vector2(5 * 4 * Math.PI, 0), module.getKinematicPosition());
	}
}
