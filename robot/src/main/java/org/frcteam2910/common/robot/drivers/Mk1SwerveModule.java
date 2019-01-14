package org.frcteam2910.common.robot.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.CAN;
import org.frcteam2910.common.Constants;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.drivers.SwerveModule;

import static org.frcteam2910.common.robot.Constants.CAN_TIMEOUT_MS;

/**
 * Driver for the 2017 revision of the 2910 swerve module
 */
public final class Mk1SwerveModule extends SwerveModule {
    private static final double TICKS_PER_INCH = 36.65;

    private final double offsetAngle;

    private final TalonSRX angleMotor;
    private final TalonSRX driveMotor;

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

    @Override
    protected double readAngle() {
        double angle = angleMotor.getSelectedSensorPosition() * ((2.0 * Math.PI) / 1024.0) + offsetAngle;
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    @Override
    public double readDistance() {
        return driveMotor.getSelectedSensorPosition(0) * (1.0 / TICKS_PER_INCH);
    }

    @Override
    protected void setTargetAngle(double angle) {
        angleMotor.set(ControlMode.Position, angle * (1024.0 / (2.0 * Math.PI)));
    }

    @Override
    public void setDriveOutput(double output) {
        driveMotor.set(ControlMode.PercentOutput, output);
    }
}
