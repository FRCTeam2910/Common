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

    public final void holonomicDrive(Vector2 translation, double rotation, boolean fieldOriented) {
        if (fieldOriented) {
            translation = translation.rotateBy(getGyroscope().getAngle());
        }

        for (SwerveModule module : getSwerveModules()) {
            Vector2 velocity = module.getModulePosition().normal().scale(rotation).add(translation);

            module.setTargetVelocity(velocity);
        }
    }

    public abstract SwerveModule[] getSwerveModules();

    @Override
    public void stop() {
        holonomicDrive(Vector2.ZERO, 0);
    }

    @Override
    public synchronized void updateKinematics(double timestamp) {
        double robotRotation = getGyroscope().getAngle().toRadians();
        double dt = timestamp - lastKinematicTimestamp;
        lastKinematicTimestamp = timestamp;

        SwerveModule[] swerveModules = getSwerveModules();

        Vector2 averagePosition = Vector2.ZERO;
        for (SwerveModule module : swerveModules) {
            module.updateSensors();
            module.updateKinematics(robotRotation);
            averagePosition = averagePosition.add(module.getCurrentPosition());
        }
        averagePosition = averagePosition.scale(1.0 / swerveModules.length);

        kinematicVelocity = averagePosition.subtract(kinematicPosition).scale(1 / (timestamp - lastKinematicTimestamp));
        kinematicPosition = averagePosition;

        for (SwerveModule module : swerveModules) {
            module.updateState(dt);
        }
    }

    public synchronized void resetKinematics(double timestamp) {
        for (SwerveModule module : getSwerveModules()) {
            module.resetKinematics(module.getModulePosition());
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
            SmartDashboard.putNumber(String.format("%s module angle", module.getName()), Math.toDegrees(module.getCurrentAngle()));
            SmartDashboard.putNumber(String.format("%s module drive distance", module.getName()), module.getCurrentDistance());
            SmartDashboard.putString(String.format("%s module position", module.getName()), module.getCurrentPosition().toString());
        }
    }
}
