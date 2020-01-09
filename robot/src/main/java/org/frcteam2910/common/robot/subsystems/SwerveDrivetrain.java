package org.frcteam2910.common.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;

import java.util.List;
import java.util.Map;
import java.util.TreeMap;

@Deprecated
public abstract class SwerveDrivetrain extends HolonomicDrivetrain {
    private Vector2 kinematicPosition = Vector2.ZERO;
    private Vector2 kinematicVelocity = Vector2.ZERO;
    private double lastKinematicTimestamp;

    private InterpolatingTreeMap<InterpolatingDouble, Vector2> positionSamples = new InterpolatingTreeMap<>(5);

    @Override
    public void holonomicDrive(Vector2 translation, double rotation, boolean fieldOriented) {
        if (fieldOriented) {
            translation = translation.rotateBy(getGyroscope().getAngle().inverse());
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

        Vector2 averageCenter = Vector2.ZERO;
        for (SwerveModule module : swerveModules) {
            module.updateSensors();
            module.updateKinematics(robotRotation);

            Vector2 estimatedCenter = new RigidTransform2(module.getCurrentPosition(),
                    Rotation2.fromRadians(robotRotation))
                    .transformBy(new RigidTransform2(module.getModulePosition().inverse(), Rotation2.ZERO)).translation;

            averageCenter = averageCenter.add(estimatedCenter);
        }
        averageCenter = averageCenter.scale(1.0 / swerveModules.length);

        positionSamples.put(new InterpolatingDouble(timestamp), averageCenter);

        {
            Map.Entry<InterpolatingDouble, Vector2> lastPosition = positionSamples.firstEntry();
            kinematicVelocity = averageCenter.subtract(lastPosition.getValue()).scale(1 / (timestamp - lastPosition.getKey().value));
        }
        kinematicPosition = averageCenter;

        for (SwerveModule module : swerveModules) {
            module.resetKinematics(new RigidTransform2(kinematicPosition, Rotation2.fromRadians(robotRotation))
                    .transformBy(new RigidTransform2(module.getModulePosition(), Rotation2.ZERO)).translation);
            module.updateState(dt);
        }
    }

    /**
     * @deprecated Use {@link #resetKinematics(Vector2, double)} instead.
     */
    @Deprecated
    @Override
    public synchronized void resetKinematics(double timestamp) {
        resetKinematics(Vector2.ZERO, timestamp);
    }

    public synchronized void resetKinematics(Vector2 position, double timestamp) {
        for (SwerveModule module : getSwerveModules()) {
            module.resetKinematics(position.add(module.getModulePosition()));
        }

        kinematicVelocity = Vector2.ZERO;
        kinematicPosition = position;
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
            SmartDashboard.putNumber(String.format("%s module velocity", module.getName()), module.getCurrentVelocity());
            SmartDashboard.putNumber(String.format("%s module drive current", module.getName()), module.getDriveCurrent() + Math.random() * MathUtils.EPSILON);
        }
    }
}
