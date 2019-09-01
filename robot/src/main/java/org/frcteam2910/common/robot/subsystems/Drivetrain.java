package org.frcteam2910.common.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.math.Vector2;

public abstract class Drivetrain extends Subsystem {
	public abstract Gyroscope getGyroscope();

	@Override
	public abstract void updateKinematics(double timestamp);

	public abstract Vector2 getKinematicPosition();

	public abstract Vector2 getKinematicVelocity();

	public void outputToSmartDashboard() {
		SmartDashboard.putString("Drivetrain position", getKinematicPosition().toString());
		SmartDashboard.putString("Drivetrain velocity", getKinematicVelocity().toString());

		SmartDashboard.putNumber("Drivetrain angle", getGyroscope().getAngle().toDegrees());
	}

	public void zeroSensors() {}
}