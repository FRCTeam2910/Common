package frc.swerverobot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.swerverobot.commands.DriveCommand;
import org.frcteam2910.common.kinematics.ChassisVelocity;
import org.frcteam2910.common.kinematics.SwerveKinematics;
import org.frcteam2910.common.kinematics.SwerveOdometry;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.UpdateManager;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModule;
import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.util.HolonomicDriveSignal;

import static frc.swerverobot.RobotMap.*;

public class DrivetrainSubsystem extends Subsystem implements UpdateManager.Updatable {
    private static final double TRACKWIDTH = 1.0;
    private static final double WHEELBASE = 1.0;

    private static final DrivetrainSubsystem instance;

    private final Mk2SwerveModule frontLeftModule = new Mk2SwerveModule(
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            DRIVETRAIN_FRONT_LEFT_MODULE_ANGLE_OFFSET,
            new Spark(DRIVETRAIN_FRONT_LEFT_MODULE_ANGLE_MOTOR),
            new CANSparkMax(DRIVETRAIN_FRONT_LEFT_MODULE_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
            new AnalogInput(DRIVETRAIN_FRONT_LEFT_MODULE_ANGLE_ENCODER)
    );
    private final Mk2SwerveModule frontRightModule = new Mk2SwerveModule(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            DRIVETRAIN_FRONT_RIGHT_MODULE_ANGLE_OFFSET,
            new Spark(DRIVETRAIN_FRONT_RIGHT_MODULE_ANGLE_MOTOR),
            new CANSparkMax(DRIVETRAIN_FRONT_RIGHT_MODULE_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
            new AnalogInput(DRIVETRAIN_FRONT_RIGHT_MODULE_ANGLE_ENCODER)
    );
    private final Mk2SwerveModule backLeftModule = new Mk2SwerveModule(
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            DRIVETRAIN_BACK_LEFT_MODULE_ANGLE_OFFSET,
            new Spark(DRIVETRAIN_BACK_LEFT_MODULE_ANGLE_MOTOR),
            new CANSparkMax(DRIVETRAIN_BACK_LEFT_MODULE_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
            new AnalogInput(DRIVETRAIN_BACK_LEFT_MODULE_ANGLE_ENCODER)
    );
    private final Mk2SwerveModule backRightModule = new Mk2SwerveModule(
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            DRIVETRAIN_BACK_RIGHT_MODULE_ANGLE_OFFSET,
            new Spark(DRIVETRAIN_BACK_RIGHT_MODULE_ANGLE_MOTOR),
            new CANSparkMax(DRIVETRAIN_BACK_RIGHT_MODULE_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
            new AnalogInput(DRIVETRAIN_BACK_RIGHT_MODULE_ANGLE_ENCODER)
    );
    private final Mk2SwerveModule[] modules = {frontLeftModule, frontRightModule, backLeftModule, backRightModule};

    private final SwerveKinematics kinematics = new SwerveKinematics(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0), // Front Left
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // Front Right
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), // Back Left
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0) // Back Right
    );
    private final SwerveOdometry odometry = new SwerveOdometry(kinematics, RigidTransform2.ZERO);

    private final Object sensorLock = new Object();
    private final NavX navX$sensorLock = new NavX(SPI.Port.kMXP);

    private final Object kinematicsLock = new Object();
    private RigidTransform2 pose$kinematicsLock = RigidTransform2.ZERO;

    private final Object stateLock = new Object();
    private HolonomicDriveSignal driveSignal$stateLock = null;

    // Logging stuff
    private NetworkTableEntry poseXEntry;
    private NetworkTableEntry poseYEntry;
    private NetworkTableEntry poseAngleEntry;

    private NetworkTableEntry[] moduleAngleEntries = new NetworkTableEntry[modules.length];

    static {
        instance = new DrivetrainSubsystem();
    }

    private DrivetrainSubsystem() {
        synchronized (sensorLock) {
            navX$sensorLock.setInverted(true);
        }

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        poseXEntry = tab.add("Pose X", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        poseYEntry = tab.add("Pose Y", 0.0)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();
        poseAngleEntry = tab.add("Pose Angle", 0.0)
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();

        ShuffleboardLayout frontLeftModuleContainer = tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withPosition(1, 0)
                .withSize(2, 3);
        moduleAngleEntries[0] = frontLeftModuleContainer.add("Angle", 0.0).getEntry();

        ShuffleboardLayout frontRightModuleContainer = tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withPosition(3, 0)
                .withSize(2, 3);
        moduleAngleEntries[1] = frontRightModuleContainer.add("Angle", 0.0).getEntry();

        ShuffleboardLayout backLeftModuleContainer = tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withPosition(5, 0)
                .withSize(2, 3);
        moduleAngleEntries[2] = backLeftModuleContainer.add("Angle", 0.0).getEntry();

        ShuffleboardLayout backRightModuleContainer = tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withPosition(7, 0)
                .withSize(2, 3);
        moduleAngleEntries[3] = backRightModuleContainer.add("Angle", 0.0).getEntry();
    }

    public static DrivetrainSubsystem getInstance() {
        return instance;
    }

    public RigidTransform2 getPose() {
        synchronized (kinematicsLock) {
            return pose$kinematicsLock;
        }
    }

    public void drive(Vector2 translationalVelocity, double rotationalVelocity, boolean fieldOriented) {
        synchronized (stateLock) {
            driveSignal$stateLock = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, fieldOriented);
        }
    }

    public void resetGyroAngle(Rotation2 angle) {
        synchronized (sensorLock) {
            navX$sensorLock.setAdjustmentAngle(
                    navX$sensorLock.getUnadjustedAngle().rotateBy(angle.inverse())
            );
        }
    }

    @Override
    public void update(double timestamp, double dt) {
        updateOdometry(dt);

        HolonomicDriveSignal driveSignal;
        synchronized (stateLock) {
            driveSignal = driveSignal$stateLock;
        }

        updateModules(driveSignal, dt);
    }

    private void updateOdometry(double dt) {
        Vector2[] moduleVelocities = new Vector2[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.updateSensors();

            moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getCurrentAngle())).scale(module.getCurrentVelocity());
        }

        Rotation2 angle;
        synchronized (sensorLock) {
            angle = navX$sensorLock.getAngle();
        }

        RigidTransform2 pose = odometry.update(angle, dt, moduleVelocities);

        synchronized (kinematicsLock) {
            pose$kinematicsLock = pose;
        }
    }

    private void updateModules(HolonomicDriveSignal signal, double dt) {
        ChassisVelocity velocity;
        if (signal == null) {
            velocity = new ChassisVelocity(Vector2.ZERO, 0.0);
        } else if (signal.isFieldOriented()) {
            velocity = new ChassisVelocity(
                    signal.getTranslation().rotateBy(getPose().rotation.inverse()),
                    signal.getRotation()
            );
        } else {
            velocity = new ChassisVelocity(signal.getTranslation(), signal.getRotation());
        }

        Vector2[] moduleOutputs = kinematics.toModuleVelocities(velocity);
        SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1.0);

        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.setTargetVelocity(moduleOutputs[i]);
            module.updateState(dt);
        }
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveCommand());
    }

    @Override
    public void periodic() {
        var pose = getPose();
        poseXEntry.setDouble(pose.translation.x);
        poseYEntry.setDouble(pose.translation.y);
        poseAngleEntry.setDouble(pose.rotation.toDegrees());

        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            moduleAngleEntries[i].setDouble(Math.toDegrees(module.getCurrentAngle()));
        }
    }
}
