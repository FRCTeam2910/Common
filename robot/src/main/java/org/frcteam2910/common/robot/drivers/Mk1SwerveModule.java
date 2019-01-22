package org.frcteam2910.common.robot.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.drivers.SwerveModule;

import static org.frcteam2910.common.robot.Constants.CAN_TIMEOUT_MS;

/**
 * Driver for the 2017 revision of the 2910 swerve module.
 * <p>
 * This implementation matches what hardware was used on both the robot we made during the 2017 off-season and what was
 * used on the 2018 competition robot. It assumes that two Talon SRXs are used to control each module over CAN and that
 * the drive Talon SRX is connected to a CIMcoder and the angle Talon SRX is connected to an analog encoder.
 * <p>
 * The drive distance units default to inches. This can be changed using
 * {@link Mk1SwerveModule#setDriveTicksPerUnit(double)}
 */
public final class Mk1SwerveModule extends SwerveModule {
    /**
     * How many angle encoder ticks occur for one radian travel.
     * <p>
     * The angle encoder travels 1 to 1 with the module so one rotation of the angle encoder is one rotation of the
     * module. A Talon SRX gives us 1024 ticks per rotation for an analog encoder so we can divide that by 2&pi; to get
     * the ticks per radian.
     */
    private static final double ANGLE_TICKS_PER_RADIAN = (1024.0 / (2.0 * Math.PI));

    /**
     * The default amount of drive encoder ticks for one unit of travel.
     * <p>
     * This value was taken from our 2018 robot.
     */
    private static final double DEFAULT_DRIVE_TICKS_PER_UNIT = 36.65;

    private final double offsetAngle;

    private final TalonSRX angleMotor;
    private final TalonSRX driveMotor;

    /**
     * The amount of drive encoder ticks that occur for one unit of travel.
     *
     * @see #DEFAULT_DRIVE_TICKS_PER_UNIT
     */
    private volatile double driveTicksPerUnit = DEFAULT_DRIVE_TICKS_PER_UNIT;

    /**
     * @param modulePosition the module's offset from the center of the robot
     * @param offsetAngle    how much to offset the angle encoder by in radians
     * @param angleMotor     the motor controller that controls the angle motor
     * @param driveMotor     the motor controller that controls the drive motor
     */
    public Mk1SwerveModule(Vector2 modulePosition,
                           double offsetAngle,
                           TalonSRX angleMotor,
                           TalonSRX driveMotor) {
        super(modulePosition);
        this.offsetAngle = offsetAngle;
        this.angleMotor = angleMotor;
        this.driveMotor = driveMotor;

        // Configure angle motor
        this.angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, CAN_TIMEOUT_MS);
        this.angleMotor.configFeedbackNotContinuous(true, CAN_TIMEOUT_MS);
        this.angleMotor.setSensorPhase(true);
        this.angleMotor.config_kP(0, 30, CAN_TIMEOUT_MS);
        this.angleMotor.config_kI(0, 0.001, CAN_TIMEOUT_MS);
        this.angleMotor.config_kD(0, 200, CAN_TIMEOUT_MS);
        this.angleMotor.config_kF(0, 0, CAN_TIMEOUT_MS);
        this.angleMotor.setNeutralMode(NeutralMode.Brake);

        // Configure drive motor
        this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, CAN_TIMEOUT_MS);
        this.driveMotor.setSensorPhase(true);
        this.driveMotor.config_kP(0, 15, CAN_TIMEOUT_MS);
        this.driveMotor.config_kI(0, 0.01, CAN_TIMEOUT_MS);
        this.driveMotor.config_kD(0, 0.1, CAN_TIMEOUT_MS);
        this.driveMotor.config_kF(0, 0.2, CAN_TIMEOUT_MS);

        // Setup current limiting
        this.driveMotor.configContinuousCurrentLimit(30, CAN_TIMEOUT_MS);
        this.driveMotor.configPeakCurrentLimit(30, CAN_TIMEOUT_MS);
        this.driveMotor.configPeakCurrentDuration(100, CAN_TIMEOUT_MS);
        this.driveMotor.enableCurrentLimit(true);
    }

    /**
     * Sets the amount of drive ticks per inch.
     * <p>
     * The amount of ticks per inch can be calculated by driving the robot some distance forwards (10 feet is usually
     * good) and then dividing the average module ticks by that distance.
     * <p>
     * The default value uses inches and should only be used for testing.
     *
     * @param driveTicksPerUnit the amount of drive ticks that occur per unit of travel
     */
    public void setDriveTicksPerUnit(double driveTicksPerUnit) {
        this.driveTicksPerUnit = driveTicksPerUnit;
    }

    @Override
    protected double readAngle() {
        double angle = angleMotor.getSelectedSensorPosition() / ANGLE_TICKS_PER_RADIAN + offsetAngle;
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    @Override
    public double readDistance() {
        return driveMotor.getSelectedSensorPosition() / driveTicksPerUnit;
    }

    @Override
    protected void setTargetAngle(double angle) {
        angleMotor.set(ControlMode.Position, angle * ANGLE_TICKS_PER_RADIAN);
    }

    @Override
    public void setDriveOutput(double output) {
        driveMotor.set(ControlMode.PercentOutput, output);
    }
}
