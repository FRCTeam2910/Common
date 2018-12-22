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
        super(positionOffset, Rotation2.fromRadians(angleOffset).inverse());

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
    protected double getAngleEncoderRotations() {
        return angleReading;
    }

    @Override
    protected void setTargetAngleRotations(double rotations) {
        synchronized (angleController) {
            angleController.setSetpoint(rotations * 2.0 * Math.PI);
        }
    }

    @Override
    protected void setDriveMotorInverted(boolean inverted) {
        driveInverted = inverted;
    }

    @Override
    public double getCurrentDrivePercentage() {
        return driveOutput;
    }

    @Override
    public void setTargetDrivePercentage(double percentage) {
        driveOutput = percentage;
    }

    @Override
    public void zeroDistance() {
        // TODO: Implement
    }

    @Override
    public double getCurrentDistance() {
        return driveReading;
    }

    @Override
    public double getCurrentRate() {
        return 0; // TODO: Implement
    }

    @Override
    public void update(double dt) {
        double currentRotation = getAngleEncoderRotations();

        synchronized (angleController) {
            angleOutput = angleController.calculate(currentRotation * 2.0 * Math.PI, dt);
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
