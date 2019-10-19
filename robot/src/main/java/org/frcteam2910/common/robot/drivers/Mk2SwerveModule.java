package org.frcteam2910.common.robot.drivers;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;

public class Mk2SwerveModule extends SwerveModule {
    /**
     * The default drive encoder rotations per unit.
     */
    public static final double DEFAULT_DRIVE_ROTATIONS_PER_UNIT = (1.0 / (4.0 * Math.PI)) * (60.0 / 15.0) * (18.0 / 26.0) * (42.0 / 14.0);

    private static final PidConstants ANGLE_CONSTANTS = new PidConstants(0.5, 0.0, 0.0001);

    private static final double CAN_UPDATE_RATE = 50.0;

    private final double angleOffset;

    private Spark steeringMotor;
    private AnalogInput angleEncoder;
    private CANSparkMax driveMotor;
    private CANEncoder driveEncoder;

    private final Object canLock = new Object();
    private double driveDistance = 0.0;
    private double drivePercentOutput = 0.0;
    private double driveVelocity = 0.0;
    private double driveCurrent = 0.0;

    private double driveEncoderRotationsPerUnit = DEFAULT_DRIVE_ROTATIONS_PER_UNIT;

    /**
     * All CAN operations are done in a separate thread to reduce latency on the control thread
     */
    private Notifier canUpdateNotifier = new Notifier(() -> {
        double driveRotations = driveEncoder.getPosition();
        synchronized (canLock) {
            driveDistance = driveRotations * (1.0 / driveEncoderRotationsPerUnit);
        }

        double driveRpm = driveEncoder.getVelocity();
        synchronized (canLock) {
            driveVelocity = driveRpm * (1.0 / 60.0) * (1.0 / driveEncoderRotationsPerUnit);
        }

        double localDriveCurrent = driveMotor.getOutputCurrent();
        synchronized (canLock) {
            driveCurrent = localDriveCurrent;
        }

        double localDrivePercentOutput;
        synchronized (canLock) {
            localDrivePercentOutput = drivePercentOutput;
        }
        driveMotor.set(localDrivePercentOutput);
    });

    private PidController angleController = new PidController(ANGLE_CONSTANTS);

    /**
     * @param modulePosition The module's offset from the center of the robot's center of rotation
     * @param angleOffset    An angle in radians that is used to offset the angle encoder
     * @param angleMotor     The motor that controls the module's angle
     * @param driveMotor     The motor that drives the module's wheel
     * @param angleEncoder   The analog input for the angle encoder
     */
    public Mk2SwerveModule(Vector2 modulePosition, double angleOffset,
                           Spark angleMotor, CANSparkMax driveMotor, AnalogInput angleEncoder) {
        super(modulePosition);
        this.angleOffset = angleOffset;
        this.steeringMotor = angleMotor;
        this.angleEncoder = angleEncoder;
        this.driveMotor = driveMotor;
        this.driveEncoder = new CANEncoder(driveMotor);

        driveMotor.setSmartCurrentLimit(60);

        angleController.setInputRange(0.0, 2.0 * Math.PI);
        angleController.setContinuous(true);
        angleController.setOutputRange(-0.5, 0.5);

        canUpdateNotifier.startPeriodic(1.0 / CAN_UPDATE_RATE);
    }

    @Override
    protected double readAngle() {
        double angle = (1.0 - angleEncoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI + angleOffset;
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    @Override
    protected double readDistance() {
        synchronized (canLock) {
            return driveDistance;
        }
    }

    protected double readVelocity() {
        synchronized (canLock) {
            return driveVelocity;
        }
    }

    protected double readDriveCurrent() {
        double localDriveCurrent;
        synchronized (canLock) {
            localDriveCurrent = driveCurrent;
        }

        return localDriveCurrent;
    }

    @Override
    public double getCurrentVelocity() {
        return readVelocity();
    }

    @Override
    public double getDriveCurrent() {
        return readDriveCurrent();
    }

    @Override
    protected void setTargetAngle(double angle) {
        angleController.setSetpoint(angle);
    }

    @Override
    protected void setDriveOutput(double output) {
        synchronized (canLock) {
            this.drivePercentOutput = output;
        }
    }

    @Override
    public void updateState(double dt) {
        super.updateState(dt);

        steeringMotor.set(angleController.calculate(getCurrentAngle(), dt));
    }

    public void setDriveEncoderRotationsPerUnit(double driveEncoderRotationsPerUnit) {
        synchronized (canLock) {
            this.driveEncoderRotationsPerUnit = driveEncoderRotationsPerUnit;
        }
    }
}
