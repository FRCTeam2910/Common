package org.frcteam2910.common.math;

/**
 * @since 0.2
 */
public class RigidTransform2 {
    public final Vector2 translation;
    public final Rotation2 rotation;

    public RigidTransform2(Vector2 translation, Rotation2 rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    private static Vector2 intersectionInternal(RigidTransform2 a, RigidTransform2 b) {
        final double t = ((a.translation.x - b.translation.x) * b.rotation.tan + b.translation.y - a.translation.y) /
                (a.rotation.sin - a.rotation.cos * b.rotation.tan);
        return a.translation.add(Vector2.fromAngle(a.rotation).scale(t));
    }

    public RigidTransform2 transformBy(RigidTransform2 other) {
        return new RigidTransform2(translation.add(other.translation.rotateBy(rotation)), rotation.rotateBy(other.rotation));
    }

    public RigidTransform2 inverse() {
        Rotation2 inverseRotation = rotation.inverse();
        return new RigidTransform2(translation.inverse().rotateBy(inverseRotation), inverseRotation);
    }

    public Vector2 intersection(RigidTransform2 other) {
        if (rotation.isParallel(other.rotation)) {
            return new Vector2(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }

        if (Math.abs(rotation.cos) < Math.abs(other.rotation.cos)) {
            return intersectionInternal(this, other);
        } else {
            return intersectionInternal(other, this);
        }
    }

    @Override
    public String toString() {
        return "{T: " + translation + ", R: " + rotation + "}";
    }
}
