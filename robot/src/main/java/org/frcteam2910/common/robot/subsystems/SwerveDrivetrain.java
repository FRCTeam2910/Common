package org.frcteam2910.common.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.Constants;

public abstract class SwerveDrivetrain extends HolonomicDrivetrain {
    private Vector2 kinematicPosition = Vector2.ZERO;
    private Vector2 kinematicVelocity = Vector2.ZERO;
    private double lastKinematicTimestamp;

    @Override
    public final void holonomicDrive(Vector2 translation, double rotation, boolean fieldOriented) {
        holonomicDrive(translation, rotation, fieldOriented, false);
    }

    public final void holonomicDrive(Vector2 translation, double rotation, boolean fieldOriented, boolean angleOnly) {
        if (fieldOriented) {
            translation = translation.rotateBy(getGyroscope().getAngle());
        }

        for (SwerveModule module : getSwerveModules()) {
            Vector2 velocity = module.getModulePosition().scale(rotation).add(translation);

            if (velocity.length > Constants.DEADBAND_RANGE || angleOnly) {
                module.setTargetAngle(velocity.getAngle());
            }

            if (!angleOnly) {
                module.setTargetDrivePercentage(velocity.length);
            }
        }
    }

    public abstract SwerveModule[] getSwerveModules();

    @Override
    public void stop() {
        holonomicDrive(Vector2.ZERO, 0);
    }

    @Override
    public void zeroSensors() {
        super.zeroSensors();
        for (SwerveModule module : getSwerveModules()) {
            module.zeroDistance();
        }
    }

    @Override
    public synchronized void updateKinematics(double timestamp) {
        Rotation2 heading = getGyroscope().getAngle();

        SwerveModule[] swerveModules = getSwerveModules();

        Vector2 averagePosition = Vector2.ZERO;
        for (SwerveModule module : swerveModules) {
            module.updateKinematics(heading);
            averagePosition = averagePosition.add(module.getKinematicPosition());
        }
        averagePosition = averagePosition.multiply(1.0 / swerveModules.length);

        kinematicVelocity = averagePosition.subtract(kinematicPosition).multiply(1 / (timestamp - lastKinematicTimestamp));
        kinematicPosition = averagePosition;
        lastKinematicTimestamp = timestamp;
    }

    public synchronized void resetKinematics(double timestamp) {
        for (SwerveModule module : getSwerveModules()) {
            module.resetKinematics(getGyroscope().getAngle());
        }
        kinematicVelocity = Vector2.ZERO;
        kinematicPosition = Vector2.ZERO;
        lastKinematicTimestamp = timestamp;
    }

    @Override
    public Vector2 getKinematicPosition() {
        return kinematicPosition;
    }

    @Override
    public Vector2 getKinematicVelocity() {
        return kinematicVelocity;
    }

    @Override
    public void outputToSmartDashboard() {
        super.outputToSmartDashboard();
        for (SwerveModule module : getSwerveModules()) {
            SmartDashboard.putNumber(String.format("%s module output %%", module.getName()), module.getCurrentDrivePercentage());
            SmartDashboard.putNumber(String.format("%s module drive distance", module.getName()), module.getCurrentDistance());
            SmartDashboard.putString(String.format("%s module position", module.getName()), module.getKinematicPosition().toString());
        }
    }
}
