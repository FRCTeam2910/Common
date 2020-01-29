package org.frcteam2910.common.robot.drivers;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class Mk2SwerveModuleBuilder {
    /**
     * The gear ratio of the angle motor that ships with the standard kit.
     */
    private static final double DEFAULT_ANGLE_REDUCTION = 18.0 / 1.0;

    /**
     * The gear ratio of the drive motor that ships with the standard kit.
     */
    private static final double DEFAULT_DRIVE_REDUCTION = 8.31 / 1.0;

    /**
     * The diameter of the standard wheel in inches.
     */
    private static final double DEFAULT_WHEEL_DIAMETER = 4.0;

    /**
     * Default constants for angle pid running on-board with NEOs.
     */
    private static final PidConstants DEFAULT_ONBOARD_NEO_ANGLE_CONSTANTS = new PidConstants(0.5, 0.0, 0.0001);

    /**
     * Default constants for angle pid running on-board with CIMs.
     */
    private static final PidConstants DEFAULT_ONBOARD_CIM_ANGLE_CONSTANTS = new PidConstants(0.5, 0.0, 0.0001);

    /**
     * Default constants for angle pid running on-board with Mini CIMs
     */
    private static final PidConstants DEFAULT_ONBOARD_MINI_CIM_ANGLE_CONSTANTS = new PidConstants(0.5, 0.0, 0.0001);

    /**
     * Default constants for angle pid running on a Spark MAX using NEOs.
     */
    private static final PidConstants DEFAULT_CAN_SPARK_MAX_ANGLE_CONSTANTS = new PidConstants(1.5, 0.0, 0.5);

    private static final PidConstants DEFAULT_FALCON_ANGLE_CONSTANTS = new PidConstants(0.1, 0.0, 0.5);

    private final Vector2 modulePosition;

    private DoubleSupplier angleSupplier;
    private DoubleSupplier currentDrawSupplier;
    private DoubleSupplier distanceSupplier;
    private DoubleSupplier velocitySupplier;

    private DoubleConsumer driveOutputConsumer;
    private DoubleConsumer targetAngleConsumer;

    private DoubleConsumer initializeAngleCallback;
    private List<BiConsumer<SwerveModule, Double>> updateCallbacks = new ArrayList<>();

    public Mk2SwerveModuleBuilder(Vector2 modulePosition) {
        this.modulePosition = modulePosition;
    }

    /**
     * Configures the swerve module to use an analog encoder through one of the RoboRIO's analog input ports.
     *
     * @param encoder The analog input handle to use for the encoder.
     * @param offset  The offset of the encoder in radians. This value is added to the analog encoder reading to obtain
     *                the true module angle.
     * @return The builder.
     */
    public Mk2SwerveModuleBuilder angleEncoder(AnalogInput encoder, double offset) {
        angleSupplier = () -> {
            double angle = (1.0 - encoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI;
            angle += offset;
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        };

        return this;
    }

    /**
     * Configures the swerve module to use a CAN Spark MAX driving a NEO as it's angle motor.
     * <p>
     * The default PID constants and angle reduction are used. These values have been determined to work with all Mk2
     * modules controlled by this motor.
     * <p>
     * To override this values see {@link #angleMotor(CANSparkMax, PidConstants, double)}
     *
     * @param motor The CAN Spark MAX to use as the angle motor. The NEO's encoder is set to output the module's angle
     *              in radians.
     * @return The builder.
     */
    public Mk2SwerveModuleBuilder angleMotor(CANSparkMax motor) {
        return angleMotor(motor, DEFAULT_CAN_SPARK_MAX_ANGLE_CONSTANTS, DEFAULT_ANGLE_REDUCTION);
    }

    public Mk2SwerveModuleBuilder angleMotor(CANSparkMax motor, MotorType motorType) {
        if (motorType == MotorType.NEO) {
            return angleMotor(motor, DEFAULT_CAN_SPARK_MAX_ANGLE_CONSTANTS, DEFAULT_ANGLE_REDUCTION);
        }

        return angleMotor((SpeedController) motor, motorType);
    }

    /**
     * Configures the swerve module to use a CAN Spark MAX driving a NEO as it's angle motor.
     * <p>
     * This method is usually used when custom PID tuning is required. If using the standard angle reduction
     * and a NEO, {@link #angleMotor(CANSparkMax)} uses already tuned constants so no tuning is required.
     *
     * @param motor     The CAN Spark MAX to use as the angle motor. The NEO's encoder is set to output the module's
     *                  angle in radians.
     * @param constants The PID constants to use to control the module's angle (units are in radians).
     * @param reduction The reduction of the angle motor. It should be specified so the number is greater than 1.
     *                  For example, an 18:1 ratio should be specified by {@code 18.0 / 1.0}.
     * @return The builder.
     */
    public Mk2SwerveModuleBuilder angleMotor(CANSparkMax motor, PidConstants constants, double reduction) {
        CANEncoder encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(2.0 * Math.PI / reduction);

        CANPIDController controller = motor.getPIDController();

        controller.setP(constants.p);
        controller.setI(constants.i);
        controller.setD(constants.i);

        targetAngleConsumer = targetAngle -> {
            double currentAngle = encoder.getPosition();
            // Calculate the current angle in the range [0, 2pi)
            double currentAngleMod = currentAngle % (2.0 * Math.PI);
            if (currentAngleMod < 0.0) {
                currentAngleMod += 2.0 * Math.PI;
            }

            // Figure out target to send to Spark MAX because the encoder is continuous
            double newTarget = targetAngle + currentAngle - currentAngleMod;
            if (targetAngle - currentAngleMod > Math.PI) {
                newTarget -= 2.0 * Math.PI;
            } else if (targetAngle - currentAngleMod < -Math.PI) {
                newTarget += 2.0 * Math.PI;
            }

            controller.setReference(newTarget, ControlType.kPosition);
        };
        initializeAngleCallback = encoder::setPosition;

        return this;
    }

    public Mk2SwerveModuleBuilder angleMotor(TalonFX motor) {
        return angleMotor(motor, DEFAULT_FALCON_ANGLE_CONSTANTS, DEFAULT_ANGLE_REDUCTION);
    }

    public Mk2SwerveModuleBuilder angleMotor(TalonFX motor, PidConstants constants, double reduction) {
        final double sensorCoefficient = (2.0 * Math.PI) / (reduction * 2048.0);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.slot0.kP = constants.p;
        config.slot0.kI = constants.i;
        config.slot0.kD = constants.d;

        motor.setNeutralMode(NeutralMode.Brake);

        motor.configAllSettings(config);

        targetAngleConsumer = targetAngle -> {
            double currentAngle = sensorCoefficient * motor.getSensorCollection().getIntegratedSensorPosition();
            // Calculate the current angle in the range [0, 2pi)
            double currentAngleMod = currentAngle % (2.0 * Math.PI);
            if (currentAngleMod < 0.0) {
                currentAngleMod += 2.0 * Math.PI;
            }

            // Figure out target to send to TalonFX because the encoder is continuous
            double newTarget = targetAngle + currentAngle - currentAngleMod;
            if (targetAngle - currentAngleMod > Math.PI) {
                newTarget -= 2.0 * Math.PI;
            } else if (targetAngle - currentAngleMod < -Math.PI) {
                newTarget += 2.0 * Math.PI;
            }

            motor.set(TalonFXControlMode.Position, newTarget / sensorCoefficient);
        };
        initializeAngleCallback = angle -> motor.getSensorCollection().setIntegratedSensorPosition(angle / sensorCoefficient, 50);

        return this;
    }

    /**
     * Configures the swerve module to use a PWM Spark MAX driving a NEO as it's angle motor.
     * <p>
     * The default PID constants are used. These values have been determined to work with all Mk2 modules
     * controlled by a NEO using the standard angle reduction.
     *
     * @param motor The PWM Spark MAX to use as the angle motor.
     * @return The builder.
     */
    public Mk2SwerveModuleBuilder angleMotor(SpeedController motor) {
        return angleMotor(motor, MotorType.NEO);
    }

    public Mk2SwerveModuleBuilder angleMotor(SpeedController motor, MotorType motorType) {
        switch (motorType) {
            case CIM:
                // Spark MAXs are special and drive brushed motors in the opposite direction of every other motor
                // controller
                if (motor instanceof Spark || motor instanceof CANSparkMax) {
                    motor.setInverted(true);
                }

                return angleMotor(motor, DEFAULT_ONBOARD_CIM_ANGLE_CONSTANTS);
            case MINI_CIM:
                // Spark MAXs are special and drive brushed motors in the opposite direction of every other motor controller
                if (motor instanceof Spark || motor instanceof CANSparkMax) {
                    motor.setInverted(true);
                }

                return angleMotor(motor, DEFAULT_ONBOARD_MINI_CIM_ANGLE_CONSTANTS);
            case NEO:
                return angleMotor(motor, DEFAULT_ONBOARD_NEO_ANGLE_CONSTANTS);
            default:
                throw new IllegalArgumentException("Unknown motor type " + motorType);
        }
    }

    /**
     * Configures the swerve module to use a generic speed controller as it's angle motor.
     * <p>
     * This method is usually used when custom PID tuning is required or a NEO is not used.
     *
     * @param motor     The speed controller to use as the angle motor.
     * @param constants The PID constants to use to control the module's angle (units are in radians).
     * @return The builder.
     */
    public Mk2SwerveModuleBuilder angleMotor(SpeedController motor, PidConstants constants) {
        PidController controller = new PidController(constants);
        controller.setInputRange(0.0, 2.0 * Math.PI);
        controller.setContinuous(true);

        targetAngleConsumer = controller::setSetpoint;
        updateCallbacks.add((module, dt) -> motor.set(controller.calculate(module.getCurrentAngle(), dt)));

        return this;
    }

    /**
     * Configures the swerve module to use a CAN Spark MAX driving a NEO as it's drive motor.
     * <p>
     * The default reduction and wheel diameter are used. These will work with the recommended wheel and stock drive
     * reduction. By default distance and velocity will be reported in inches and inches per second.
     *
     * @param motor The CAN Spark MAX to use as the drive motor. The NEO's encoder is set to output the module's driven
     *              distance and current velocity in inches and inches per second.
     * @return The builder.
     */
    public Mk2SwerveModuleBuilder driveMotor(CANSparkMax motor) {
        return driveMotor(motor, MotorType.NEO);
    }

    public Mk2SwerveModuleBuilder driveMotor(CANSparkMax motor, MotorType motorType) {
        if (motorType == MotorType.NEO) {
            return driveMotor(motor, DEFAULT_DRIVE_REDUCTION, DEFAULT_WHEEL_DIAMETER);
        }

        return driveMotor((SpeedController) motor, motorType);
    }

    /**
     * Configures the swerve module to use a CAN Spark MAX driving a NEO as it's drive motor.
     *
     * @param motor         The CAN Spark MAX to use as the drive motor. The NEO's encoder is set to output the module's driven
     *                      distance and current velocity. Units are the same as the units used for the wheel diameter.
     * @param reduction     The drive reduction of the module. It should be specified so the number is greater than 1.
     *                      For example, an 18:1 ratio should be specified by {@code 18.0 / 1.0}.
     * @param wheelDiameter The diameter of the module's wheel. By default this is {@value DEFAULT_WHEEL_DIAMETER}
     *                      inches.
     * @return The builder.
     */
    public Mk2SwerveModuleBuilder driveMotor(CANSparkMax motor, double reduction, double wheelDiameter) {
        CANEncoder encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(wheelDiameter * Math.PI / reduction);
        encoder.setVelocityConversionFactor(wheelDiameter * Math.PI / reduction * (1.0 / 60.0)); // RPM to units per second

        currentDrawSupplier = motor::getOutputCurrent;
        distanceSupplier = encoder::getPosition;
        velocitySupplier = encoder::getVelocity;
        driveOutputConsumer = motor::set;

        return this;
    }

    public Mk2SwerveModuleBuilder driveMotor(TalonFX motor) {
        return driveMotor(motor, DEFAULT_DRIVE_REDUCTION, DEFAULT_WHEEL_DIAMETER);
    }

    public Mk2SwerveModuleBuilder driveMotor(TalonFX motor, double reduction, double wheelDiameter) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        motor.configAllSettings(config);
        motor.setNeutralMode(NeutralMode.Brake);

        currentDrawSupplier = motor::getSupplyCurrent;
        distanceSupplier = () -> (Math.PI * wheelDiameter * motor.getSensorCollection().getIntegratedSensorPosition()) / (2048.0 * reduction);
        velocitySupplier = () -> (10.0 * Math.PI * wheelDiameter * motor.getSensorCollection().getIntegratedSensorVelocity()) / (2048.0 * reduction);
        driveOutputConsumer = output -> motor.set(TalonFXControlMode.PercentOutput, output);

        return this;
    }

    /**
     * Configures the swerve module to use a generic speed controller driving the specified motor.
     *
     * @param motor     The speed controller to use.
     * @param motorType The type of motor used.
     * @return The builder.
     */
    public Mk2SwerveModuleBuilder driveMotor(SpeedController motor, MotorType motorType) {
        // Spark MAXs are special and drive brushed motors in the opposite direction of every other motor controller
        if (motorType != MotorType.NEO && (motor instanceof Spark || motor instanceof CANSparkMax)) {
            motor.setInverted(true);
        }

        driveOutputConsumer = motor::set;

        return this;
    }

    /**
     * Builds and returns a configured swerve module.
     *
     * @return The built swerve module.
     */
    public SwerveModule build() {
        // Verify everything is populated
        if (angleSupplier == null) {
            // Absolute angle encoder not configured
            throw new IllegalStateException("No absolute encoder has been configured! See Mk2SwerveModuleBuilder.angleEncoder");
        } else if (driveOutputConsumer == null) {
            // Drive motor not configured
            throw new IllegalStateException("No drive motor has been configured! See Mk2SwerveModuleBuilder.driveMotor");
        } else if (targetAngleConsumer == null) {
            // Angle motor not configured
            throw new IllegalStateException("No angle motor has been configured! See Mk2SwerveModuleBuilder.angleMotor");
        }

        return new SwerveModuleImpl();
    }

    private final class SwerveModuleImpl extends SwerveModule {
        private final Object sensorLock = new Object();
        private double currentDraw = 0.0;
        private double velocity = 0.0;

        public SwerveModuleImpl() {
            super(modulePosition);

            if (initializeAngleCallback != null) {
                initializeAngleCallback.accept(angleSupplier.getAsDouble());
            }
        }

        @Override
        protected double readAngle() {
            return angleSupplier.getAsDouble();
        }

        protected double readCurrentDraw() {
            if (currentDrawSupplier == null) {
                return Double.NaN;
            }

            return currentDrawSupplier.getAsDouble();
        }

        @Override
        protected double readDistance() {
            if (distanceSupplier == null) {
                return Double.NaN;
            }

            return distanceSupplier.getAsDouble();
        }

        protected double readVelocity() {
            if (velocitySupplier == null) {
                return Double.NaN;
            }

            return velocitySupplier.getAsDouble();
        }

        @Override
        public double getCurrentVelocity() {
            synchronized (sensorLock) {
                return velocity;
            }
        }

        @Override
        public double getDriveCurrent() {
            synchronized (sensorLock) {
                return currentDraw;
            }
        }

        @Override
        protected void setTargetAngle(double angle) {
            targetAngleConsumer.accept(angle);
        }

        @Override
        protected void setDriveOutput(double output) {
            driveOutputConsumer.accept(output);
        }

        @Override
        public void updateSensors() {
            super.updateSensors();

            double newCurrentDraw = readCurrentDraw();
            double newVelocity = readVelocity();

            synchronized (sensorLock) {
                currentDraw = newCurrentDraw;
                velocity = newVelocity;
            }
        }

        @Override
        public void updateState(double dt) {
            super.updateState(dt);

            updateCallbacks.forEach(c -> c.accept(this, dt));
        }
    }

    public enum MotorType {
        CIM,
        MINI_CIM,
        NEO,
        FALCON_500
    }
}
