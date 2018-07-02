package org.frcteam2910.common.robot.drivers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Vector2;

public final class Limelight {
	private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

	private final NetworkTableEntry tv = table.getEntry("tv");
	private final NetworkTableEntry tx = table.getEntry("tx");
	private final NetworkTableEntry ty = table.getEntry("ty");
	private final NetworkTableEntry ta = table.getEntry("ta");
	private final NetworkTableEntry ts = table.getEntry("ts");
	private final NetworkTableEntry tl = table.getEntry("tl");

	private final NetworkTableEntry ledMode = table.getEntry("ledMode");
	private final NetworkTableEntry camMode = table.getEntry("camMode");
	private final NetworkTableEntry pipeline = table.getEntry("pipeline");
	private final NetworkTableEntry stream = table.getEntry("stream");
	private final NetworkTableEntry snapshot = table.getEntry("snapshot");

	public boolean hasTarget() {
		return MathUtils.epsilonEquals(tv.getDouble(0), 1);
	}

	public double getTargetArea() {
		return ta.getDouble(0);
	}

	public Vector2 getTargetPosition() {
		return new Vector2(tx.getDouble(0), ty.getDouble(0));
	}

	public double getTargetSkew() {
		return ts.getDouble(0);
	}

	public void setCamMode(CamMode mode) {
		switch (mode) {
			case VISON:
				camMode.setNumber(0);
				break;
			case DRIVER:
				camMode.setNumber(1);
		}
	}

	public void setLedMode(LedMode mode) {
		switch (mode) {
			case ON:
				ledMode.setNumber(0);
				break;
			case OFF:
				ledMode.setNumber(1);
				break;
			case BLINK:
				ledMode.setNumber(2);
		}
	}

	public void setSnapshotsEnabled(boolean enabled) {
		if (enabled)
			snapshot.setNumber(1);
		else
			snapshot.setNumber(0);
	}

	public void setPipeline(int pipeline) {
		this.pipeline.setNumber(pipeline);
	}

	public void setStreamMode(StreamMode mode) {
		switch (mode) {
			case STANDARD:
				stream.setNumber(0);
				break;
			case PIP_MAIN:
				stream.setNumber(1);
				break;
			case PIP_SECONDARY:
				stream.setNumber(2);
				break;
		}
	}

	public enum CamMode {
		VISON,
		DRIVER
	}

	public enum LedMode {
		ON, OFF, BLINK
	}

	public enum StreamMode {
		STANDARD,
		PIP_MAIN,
		PIP_SECONDARY
	}
}
