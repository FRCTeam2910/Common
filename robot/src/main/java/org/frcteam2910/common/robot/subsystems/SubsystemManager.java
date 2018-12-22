package org.frcteam2910.common.robot.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class SubsystemManager {
	private final List<Subsystem> subsystems = new ArrayList<>();

	private double lastTimestamp = 0.0;

	private final Notifier kinematicThread = new Notifier(() -> {
		synchronized (SubsystemManager.this) {
			final double timestamp = Timer.getFPGATimestamp();
			final double dt = timestamp - lastTimestamp;
			lastTimestamp = timestamp;
			SmartDashboard.putNumber("Updater rate", 1.0 / dt);
			subsystems.forEach(s -> s.updateKinematics(timestamp));
		}
	});

	public SubsystemManager(Subsystem... subsystems) {
		this(Arrays.asList(subsystems));
	}

	public SubsystemManager(List<Subsystem> subsystems) {
		this.subsystems.addAll(subsystems);
	}

	public void enableKinematicLoop(double period) {
		final double timestamp = Timer.getFPGATimestamp();
		for (Subsystem subsystem : subsystems) {
			subsystem.resetKinematics(timestamp);
		}
		kinematicThread.startPeriodic(period);
	}

	public void disableKinematicLoop() {
		kinematicThread.stop();
	}

	public void outputToSmartDashboard() {
		subsystems.forEach(Subsystem::outputToSmartDashboard);
	}

	public void writeToLog() {
		subsystems.forEach(Subsystem::writeToLog);
	}

	public void stop() {
		subsystems.forEach(Subsystem::stop);
	}

	public void zeroSensors() {
		subsystems.forEach(Subsystem::zeroSensors);
	}
}
