package org.frcteam2910.common.robot.drivers;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.Constants;

/**
 * A driver for the 2017 swerve module hardware but using onboard pid.
 */
public class OnboardPid2017SwerveModule extends OnboardPidSwerveModule {
    private static final PidConstants ANGLE_PID_CONSTANTS = new PidConstants(0.5, 0.0, 0.0);
    private static final double DRIVE_TICKS_PER_INCH = 36.65;

    public OnboardPid2017SwerveModule(Vector2 positionOffset, double angleOffset,
                                      WPI_TalonSRX angleMotor, WPI_TalonSRX driveMotor) {
        super(positionOffset, angleOffset,
                angleMotor, driveMotor,
                () -> angleMotor.getSelectedSensorPosition(0) * ((2.0 * Math.PI) / 1024.0),
                () -> driveMotor.getSelectedSensorPosition(0) / DRIVE_TICKS_PER_INCH,
                ANGLE_PID_CONSTANTS);

        angleMotor.setInverted(true);
        angleMotor.setSensorPhase(true);
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, Constants.CAN_TIMEOUT_MS);
        angleMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.setSensorPhase(true);
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT_MS);
        angleMotor.setNeutralMode(NeutralMode.Coast);
        driveMotor.configContinuousCurrentLimit(30, Constants.CAN_TIMEOUT_MS);
        driveMotor.configPeakCurrentLimit(0, Constants.CAN_TIMEOUT_MS); // Disable peak current limiting
        driveMotor.enableCurrentLimit(true);
    }
}
