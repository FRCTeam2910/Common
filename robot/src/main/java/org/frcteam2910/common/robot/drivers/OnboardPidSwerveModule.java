package org.frcteam2910.common.robot.drivers;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.util.function.DoubleSupplier;

public class OnboardPidSwerveModule extends SwerveModule {
    /**
     * The default hardware update rate in hertz
     */
    public static final double DEFAULT_HARDWARE_UPDATE_RATE = 200.0;

    private final double angleOffset;

    private final SpeedController angleMotor;
    private final SpeedController driveMotor;

    private final DoubleSupplier angleEncoder;
    private final DoubleSupplier driveEncoder;

    private final PidController angleController;

    private final Notifier hardwareUpdateThread;

    private volatile boolean driveInverted = false;

    private volatile double angleReading;
    private volatile double driveReading;

    private volatile double angleOutput;
    private volatile double driveOutput;

    private volatile double currentHardwareUpdateRate;

    /**
     * @param positionOffset    the modules offset from the robot's center of rotation in inches
     * @param angleOffset       the angle offset of the module in radians
     * @param angleMotor        the angle motor speed controller
     * @param driveMotor        the drive motor speed controller
     * @param angleEncoder      a supplier that will return the module angle encoder value in radians
     * @param driveEncoder      a supplier that will return the module drive encoder value in inches
     * @param anglePidConstants the pid constants used for the the angle pid controller
     */
    public OnboardPidSwerveModule(Vector2 positionOffset, double angleOffset,
                                  SpeedController angleMotor, SpeedController driveMotor,
                                  DoubleSupplier angleEncoder, DoubleSupplier driveEncoder,
                                  PidConstants anglePidConstants) {
        super(positionOffset);
        this.angleOffset = angleOffset;

        this.angleMotor = angleMotor;
        this.driveMotor = driveMotor;
        this.angleEncoder = angleEncoder;
        this.driveEncoder = driveEncoder;

        this.angleController = new PidController(anglePidConstants);
        angleController.setContinuous(true);
        angleController.setInputRange(0.0, 2.0 * Math.PI);

        this.hardwareUpdateThread = new Notifier(new HardwareUpdater());
        hardwareUpdateThread.startPeriodic(1.0 / DEFAULT_HARDWARE_UPDATE_RATE);
    }

    /**
     * Sets the hardware update thread rate. The default rate is {@value #DEFAULT_HARDWARE_UPDATE_RATE}.
     *
     * @param updateRate the rate in hertz that the hardware update thread should run.
     */
    public void setHardwareUpdateRate(double updateRate) {
        hardwareUpdateThread.startPeriodic(1.0 / updateRate);
    }

    /**
     * Gets the current rate that the hardware update thread is running at. This is not the rate specified by
     * {@link #setHardwareUpdateRate(double)} but instead is determined by how fast the hardware thread is currently
     * running.
     *
     * @return the rate the hardware update thread is running in hertz
     */
    public double getCurrentHardwareUpdateRate() {
        return currentHardwareUpdateRate;
    }

    /**
     * Gets the PID controller that controls the module angle.
     * <p>
     * This should only be called by the constructor.
     *
     * @return the angle controller
     */
    protected PidController getAngleController() {
        return angleController;
    }

    @Override
    protected double readAngle() {
        double angle = angleReading + angleOffset % 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    @Override
    protected void setTargetAngle(double angle) {
        synchronized (angleController) {
            angleController.setSetpoint(angle);
        }
    }

    @Override
    public void setDriveOutput(double output) {
        driveOutput = output;
    }

    @Override
    public double readDistance() {
        return driveReading;
    }

    @Override
    public void updateState(double dt) {
        super.updateState(dt);

        double currentAngle = getCurrentAngle();

        synchronized (angleController) {
            angleOutput = angleController.calculate(currentAngle, dt);
        }
    }

    private class HardwareUpdater implements Runnable {
        private double lastUpdateTime = 0.0;

        @Override
        public void run() {
            double now = Timer.getFPGATimestamp();
            double dt = now - lastUpdateTime;
            lastUpdateTime = now;
            currentHardwareUpdateRate = 1.0 / dt;

            // Read the encoder values
            angleReading = angleEncoder.getAsDouble() / (2.0 * Math.PI);
            driveReading = driveEncoder.getAsDouble();

            // Write motor values
            angleMotor.set(angleOutput);
            driveMotor.set(driveOutput * (driveInverted ? -1.0 : 1.0));
        }
    }
}
