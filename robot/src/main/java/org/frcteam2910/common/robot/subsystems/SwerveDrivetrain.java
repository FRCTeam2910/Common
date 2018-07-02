package org.frcteam2910.common.robot.subsystems;

import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.Constants;

public abstract class SwerveDrivetrain extends HolonomicDrivetrain {
    private Vector2 kinematicPosition = new Vector2();
    private Vector2 kinematicVelocity = new Vector2();
    private double lastKinematicTimestamp;

    @Override
    public final void holonomicDrive(Vector2 translation, double rotation, boolean fieldOriented) {
        holonomicDrive(translation, rotation, fieldOriented, false);
    }

    public final void holonomicDrive(Vector2 translation, double rotation, boolean fieldOriented, boolean angleOnly) {
        if (fieldOriented) {
            double gyroRadians = Math.toRadians(getGyroscope().getAngle());
            translation = translation.rotateBy(gyroRadians);
        }

        for (SwerveModule module : getSwerveModules()) {
            Vector2 velocity = module.getModulePosition().multiply(rotation).add(translation);

            if (velocity.length > Constants.DEADBAND_RANGE || angleOnly) {
                module.setTargetAngle(Math.toDegrees(velocity.angle));
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
        double heading = getGyroscope().getAngle();

        SwerveModule[] swerveModules = getSwerveModules();

        Vector2 averagePosition = new Vector2();
        for (SwerveModule module : swerveModules) {
            module.updateKinematics(heading);
            averagePosition.add(module.getKinematicPosition());
        }
        averagePosition.multiply(1.0 / swerveModules.length);

        kinematicVelocity = averagePosition.subtract(kinematicPosition).multiply(1 / (timestamp - lastKinematicTimestamp));
        kinematicPosition = averagePosition;
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
//			module.outputToSmartDashboard();
        }
    }
}
