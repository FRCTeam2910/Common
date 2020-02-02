package org.frcteam2910.common.robot.drivers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Vector2;

public final class Limelight {
    private final NetworkTable table;

    private final NetworkTableEntry tv;
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry ta;
    private final NetworkTableEntry ts;
    private final NetworkTableEntry tl;

    private final NetworkTableEntry tcornx;
    private final NetworkTableEntry tcorny;

    private final NetworkTableEntry ledMode;
    private final NetworkTableEntry camMode;
    private final NetworkTableEntry pipeline;
    private final NetworkTableEntry stream;
    private final NetworkTableEntry snapshot;

    /**
     * Creates an instance of the Limelight, assuming the name is "limelight".
     */
    public Limelight() {
        this(NetworkTableInstance.getDefault().getTable("limelight"));
    }

    /**
     * Creates an instance of the Limelight class given the name of the Limelight. For example,
     * if the Limelight is called "limelight-cargo", pass in "cargo".
     * @param name The name of the Limelight.
     */
    public Limelight(String name) {
        this(NetworkTableInstance.getDefault().getTable("limelight-" + name));
    }

    /**
     * Creates an instance of the Limelight class given its NetworkTable.
     * @param table The NetworkTable used to create the Limelight.
     */
    public Limelight(NetworkTable table) {
        this.table = table;

        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        ts = table.getEntry("ts");
        tl = table.getEntry("tl");

        tcornx = table.getEntry("tcornx");
        tcorny = table.getEntry("tcorny");

        ledMode = table.getEntry("ledMode");
        camMode = table.getEntry("camMode");
        pipeline = table.getEntry("pipeline");
        stream = table.getEntry("stream");
        snapshot = table.getEntry("snapshot");
    }

    /**
     * Checks if the method has a target.
     * @returns Whether the Limelight has a target.
     */
    public boolean hasTarget() {
        return MathUtils.epsilonEquals(tv.getDouble(0), 1);
    }

    /**
     * Gets the area of the target divided by the size of the image.
     * @returns A value from 0.0 to 1.0 representing the target area.
     */
    public double getTargetArea() {
        return ta.getDouble(0);
    }

    /**
     * Gets the position of the target in radians within the image.
     * @returns The position of the target.
     */
    public Vector2 getTargetPosition() {
        return new Vector2(Math.toRadians(tx.getDouble(0)), Math.toRadians(ty.getDouble(0)));
    }

    /**
     * Gets the target's skew or rotation in degrees.
     * @returns The target's skew from -90 to 0 in degrees.
     */
    public double getTargetSkew() {
        return ts.getDouble(0);
    }

    /**
     * Gets the latency of the pipeline in ms (milliseconds).
     * @returns The latency of the pipeline in ms.
     */
    public double getPipelineLatency() {
        return tl.getDouble(0.0);
    }

    /**
     * Gets the vertices of the target.
     * @returns The vertices of the target.
     */
    public double[][] getCorners() {
        double[] x = tcornx.getDoubleArray(new double[]{0.0, 0.0});
        double[] y = tcorny.getDoubleArray(new double[]{0.0, 0.0});
        double[][] corners = new double[x.length][2];
        for (int i = 0; i < x.length; i++) {
            corners[i][0] = x[i];
            corners[i][1] = y[i];
        }
        return corners;
    }

    /**
     * Sets the operating mode of the Limelight.
     * @param mode The operating mode the Limelight should be set to.
     */
    public void setCamMode(CamMode mode) {
        switch (mode) {
            case VISION:
                camMode.setNumber(0);
                break;
            case DRIVER:
                camMode.setNumber(1);
        }
    }

    /**
     * Sets the mode of the LED's of the Limelight.
     * @param mode The mode the LED's should be set to.
     */
    public void setLedMode(LedMode mode) {
        switch (mode) {
            case DEFAULT:
                ledMode.setNumber(0);
                break;
            case OFF:
                ledMode.setNumber(1);
                break;
            case BLINK:
                ledMode.setNumber(2);
                break;
            case ON:
                ledMode.setNumber(3);
                break;
        }
    }

    /**
     * Sets whether you want the Limelight to take snapshots or not. When it's taking snapshots, it will do it
     * twice every second.
     * @param enabled Whether snapshots should be enabled.
     */
    public void setSnapshotsEnabled(boolean enabled) {
        if (enabled) {
            snapshot.setNumber(1);
        } else {
            snapshot.setNumber(0);
        }
    }

    /**
     * Sets which pipeline you want to use (0-9).
     * @param pipeline The index of the desired pipeline.
     */
    public void setPipeline(int pipeline) {
        this.pipeline.setNumber(pipeline);
    }

    /**
     * Changes what the Limelight streams.
     * @param mode What the Limelight should be streaming.
     */
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

    /**
     * Gets the NetworkTable being used to get values from the Limelight.
     */
    public NetworkTable getTable() {
        return table;
    }

    /**
     * Represents the different operating modes of the Limelight.
     */
    public enum CamMode {
        /**
         * Brings the exposure down and runs the pipeline
         */
        VISION,
        /**
         * Disables the pipeline, and brings the exposure up
         */
        DRIVER
    }

    /**
     * Represents the different LED modes of the Limelight
     */
    public enum LedMode {
        /**
         * Sets the LED's to whatever is specified in the pipeline
         */
        DEFAULT,
        /**
         * Turns the LED's on
         */
        ON,
        /**
         * Turns the LED's off
         */
        OFF,
        /**
         * Makes the LED's blink
         */
        BLINK
    }

    /**
     * Represents the different streaming modes of the camera
     */
    public enum StreamMode {
        /**
         * Side-by-side streams if a webcam is attached to Limelight
         */
        STANDARD,
        /**
         * The secondary camera stream is placed in the lower-right corner of the primary camera stream
         */
        PIP_MAIN,
        /**
         * The primary camera stream is placed in the lower-right corner of the secondary camera stream
         */
        PIP_SECONDARY
    }
}
