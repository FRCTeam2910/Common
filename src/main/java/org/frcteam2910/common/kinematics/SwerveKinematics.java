package org.frcteam2910.common.kinematics;

import org.ejml.simple.SimpleMatrix;
import org.frcteam2910.common.math.Vector2;

import java.util.Arrays;

public class SwerveKinematics {
    private final Vector2[] moduleOffsets;

    private final SimpleMatrix inverseKinematics;
    private final SimpleMatrix forwardKinematics;

    public SwerveKinematics(Vector2... moduleOffsets) {
        if (moduleOffsets.length < 1) {
            throw new IllegalArgumentException("Must have at least 1 module");
        }

        this.moduleOffsets = Arrays.copyOf(moduleOffsets, moduleOffsets.length);

        inverseKinematics = new SimpleMatrix(moduleOffsets.length * 2, 3);
        for (int i = 0; i < moduleOffsets.length; i++) {
            inverseKinematics.setRow(i * 2 + 0, 0, 1.0, 0.0, -moduleOffsets[i].y);
            inverseKinematics.setRow(i * 2 + 1, 0, 0.0, 1.0, moduleOffsets[i].x);
        }
        forwardKinematics = inverseKinematics.pseudoInverse();
    }

    public Vector2[] toModuleVelocities(ChassisVelocity velocity) {
        SimpleMatrix chassisVelocityVector = new SimpleMatrix(3, 1);
        chassisVelocityVector.setColumn(0, 0,
                velocity.getTranslationalVelocity().x,
                velocity.getTranslationalVelocity().y,
                velocity.getRotationalVelocity());

        SimpleMatrix moduleVelocitiesMatrix = inverseKinematics.mult(chassisVelocityVector);
        Vector2[] moduleVelocities = new Vector2[moduleOffsets.length];

        for (int i = 0; i < moduleOffsets.length; i++) {
            moduleVelocities[i] = new Vector2(
                    moduleVelocitiesMatrix.get(i * 2 + 0),
                    moduleVelocitiesMatrix.get(i * 2 + 1)
            );
        }

        return moduleVelocities;
    }

    public ChassisVelocity toChassisVelocity(Vector2... moduleVelocities) {
        if (moduleVelocities.length != moduleOffsets.length) {
            throw new IllegalArgumentException("Amount of module velocities given does not match the amount of modules specified in the constructor");
        }

        SimpleMatrix moduleVelocitiesMatrix = new SimpleMatrix(moduleOffsets.length * 2, 1);
        for (int i = 0; i < moduleOffsets.length; i++) {
            moduleVelocitiesMatrix.setColumn(0, i * 2,
                    moduleVelocities[i].x,
                    moduleVelocities[i].y
            );
        }

        SimpleMatrix chassisVelocityVector = forwardKinematics.mult(moduleVelocitiesMatrix);
        return new ChassisVelocity(
                new Vector2(
                        chassisVelocityVector.get(0),
                        chassisVelocityVector.get(1)
                ),
                chassisVelocityVector.get(2)
        );
    }

    public static void normalizeModuleVelocities(Vector2[] moduleVelocities, double maximumVelocity) {
        double realMaxVelocity = Arrays.stream(moduleVelocities).mapToDouble(m -> m.length).max().orElseThrow();
        if (realMaxVelocity > maximumVelocity) {
            for (int i = 0; i < moduleVelocities.length; i++) {
                moduleVelocities[i] = moduleVelocities[i].scale(maximumVelocity / realMaxVelocity);
            }
        }
    }
}
