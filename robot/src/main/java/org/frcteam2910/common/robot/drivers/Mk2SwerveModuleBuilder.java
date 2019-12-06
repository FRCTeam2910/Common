package org.frcteam2910.common.robot.drivers;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
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
     * Default constants for angle pid running on-board when running NEOs.
     */
    private static final PidConstants DEFAULT_ONBOARD_ANGLE_CONSTANTS = new PidConstants(0.5, 0.0, 0.0001);

    /**
     * Default constants for angle pid running on a Spark MAX using NEOs.
     */
    private static final PidConstants DEFAULT_CAN_SPARK_MAX_ANGLE_CONSTANTS = new PidConstants(1.5, 0.0, 0.5);

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

    public Mk2SwerveModuleBuilder angleMotor(CANSparkMax motor) {
        return angleMotor(motor, DEFAULT_CAN_SPARK_MAX_ANGLE_CONSTANTS, DEFAULT_ANGLE_REDUCTION);
    }

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

    public Mk2SwerveModuleBuilder angleMotor(SpeedController motor) {
        return angleMotor(motor, DEFAULT_ONBOARD_ANGLE_CONSTANTS);
    }

    public Mk2SwerveModuleBuilder angleMotor(SpeedController motor, PidConstants constants) {
        PidController controller = new PidController(constants);
        controller.setInputRange(0.0, 2.0 * Math.PI);
        controller.setContinuous(true);

        targetAngleConsumer = controller::setSetpoint;
        updateCallbacks.add((module, dt) -> motor.set(controller.calculate(module.getCurrentAngle(), dt)));

        return this;
    }

    public Mk2SwerveModuleBuilder driveMotor(CANSparkMax motor) {
        return driveMotor(motor, DEFAULT_DRIVE_REDUCTION, DEFAULT_WHEEL_DIAMETER);
    }

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

    public SwerveModule build() {
        // Verify everything is populated
        if (angleSupplier == null) {
            // Absolute angle encoder not configured
            throw new IllegalStateException(""); // TODO: Exception message
        } else if (currentDrawSupplier == null) {
            // Current draw not configured
            throw new IllegalStateException(""); // TODO: Exception message
        } else if (distanceSupplier == null || velocitySupplier == null) {
            // Drive encoder not configured
            throw new IllegalStateException(""); // TODO: Exception message
        } else if (driveOutputConsumer == null) {
            // Drive motor not configured
            throw new IllegalStateException(""); // TODO: Exception message
        } else if (targetAngleConsumer == null) {
            // Angle motor not configured
            throw new IllegalStateException(""); // TODO: Exception message
        }

        return new SwerveModuleImpl();
    }

    private final class SwerveModuleImpl extends SwerveModule {
        private final Object sensorLock = new Object();
        private double currentDraw = 0.0;
        private double velocity = 0.0;

        public SwerveModuleImpl() {
            super(modulePosition);

            initializeAngleCallback.accept(angleSupplier.getAsDouble());
        }

        @Override
        protected double readAngle() {
            return angleSupplier.getAsDouble();
        }

        protected double readCurrentDraw() {
            return currentDrawSupplier.getAsDouble();
        }

        @Override
        protected double readDistance() {
            return distanceSupplier.getAsDouble();
        }

        protected double readVelocity() {
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
}
