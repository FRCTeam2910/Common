package org.frcteam2910.common.robot.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class SubsystemManager {
	private final List<Updatable> updatables = new ArrayList<>();

	@FunctionalInterface
	private interface Updatable {
		void update(double time, double dt);
	}

	private double lastTimestamp = 0.0;

	private final Notifier kinematicThread = new Notifier(() -> {
		synchronized (SubsystemManager.this) {
			final double timestamp = Timer.getFPGATimestamp();
			final double dt = timestamp - lastTimestamp;
			lastTimestamp = timestamp;
			SmartDashboard.putNumber("Updater rate", 1.0 / dt);
			updatables.forEach(s -> s.update(timestamp, dt));
		}
	});

	public SubsystemManager(Updatable... updatables) {
		this(Arrays.asList(updatables));
	}

	public SubsystemManager(List<Updatable> subsystems) {
		this.updatables.addAll(subsystems);
	}

	public void enableKinematicLoop(double period) {
		final double timestamp = Timer.getFPGATimestamp();
		kinematicThread.startPeriodic(period);
	}

	public void disableKinematicLoop() {
		kinematicThread.stop();
	}
}
