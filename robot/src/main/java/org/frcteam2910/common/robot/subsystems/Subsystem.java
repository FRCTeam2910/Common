package org.frcteam2910.common.robot.subsystems;

public abstract class Subsystem extends edu.wpi.first.wpilibj.command.Subsystem {

	public void writeToLog() {}

	public void updateKinematics(double timestamp) {}

	public void resetKinematics(double timestamp) {}

	public abstract void outputToSmartDashboard();

	public abstract void stop();

	public abstract void zeroSensors();
}
