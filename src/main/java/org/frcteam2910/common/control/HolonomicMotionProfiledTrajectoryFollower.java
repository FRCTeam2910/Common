package org.frcteam2910.common.control;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Vector2;

public class HolonomicMotionProfiledTrajectoryFollower extends TrajectoryFollower<HolonomicMotionProfiledTrajectoryFollower.DriveSignal> {
    private PidController forwardController;
    private PidController strafeController;
    private PidController rotationController;

    private final double translationKv;
    private final double translationKa;
    private final double translationKs;

    public HolonomicMotionProfiledTrajectoryFollower(PidConstants translationConstants, PidConstants rotationConstants,
                                                     double translationKv, double translationKa, double translationKs) {
        forwardController = new PidController(translationConstants);
        strafeController = new PidController(translationConstants);
        rotationController = new PidController(rotationConstants);

        this.translationKv = translationKv;
        this.translationKa = translationKa;
        this.translationKs = translationKs;
    }

    @Override
    protected DriveSignal calculateDriveSignal(RigidTransform2 currentPose, Vector2 velocity,
                                               double rotationalVelocity, Trajectory trajectory, double time,
                                               double dt) {
        Trajectory.Segment currentSegment = trajectory.calculateSegment(time);

        Vector2 segmentVelocity = Vector2.fromAngle(currentSegment.heading).scale(currentSegment.velocity);
        Vector2 segmentAcceleration = Vector2.fromAngle(currentSegment.heading).scale(currentSegment.acceleration);

        double forwardFeedforward = translationKv * segmentVelocity.x + translationKa * segmentAcceleration.x;
        double strafeFeedforward = translationKv * segmentVelocity.y + translationKa * segmentAcceleration.y;
        {
            // Apply the Ks constant proportionally to the forward and strafe feed forwards based on their relative
            // magnitude
            Vector2 feedforwardUnitVector = new Vector2(forwardFeedforward, strafeFeedforward).normal();
            forwardFeedforward += Math.copySign(feedforwardUnitVector.x * translationKs, forwardFeedforward);
            strafeFeedforward += Math.copySign(feedforwardUnitVector.y * translationKs, strafeFeedforward);
        }

        forwardController.setSetpoint(currentSegment.translation.x);
        strafeController.setSetpoint(currentSegment.translation.y);
        rotationController.setSetpoint(currentSegment.rotation.toRadians());

        return new DriveSignal(
                new Vector2(
                        forwardController.calculate(currentPose.translation.x, dt) + forwardFeedforward,
                        strafeController.calculate(currentPose.translation.y, dt) + strafeFeedforward
                ),
                rotationController.calculate(currentPose.rotation.toRadians(), dt),
                true
        );
    }

    @Override
    protected void reset() {
        forwardController.reset();
        strafeController.reset();
        rotationController.reset();
    }

    public static class DriveSignal {
        private final Vector2 translation;
        private final double rotation;
        private final boolean fieldOriented;

        public DriveSignal(Vector2 translation, double rotation) {
            this(translation, rotation, true);
        }

        public DriveSignal(Vector2 translation, double rotation, boolean fieldOriented) {
            this.translation = translation;
            this.rotation = rotation;
            this.fieldOriented = fieldOriented;
        }

        public Vector2 getTranslation() {
            return translation;
        }

        public double getRotation() {
            return rotation;
        }

        public boolean isFieldOriented() {
            return fieldOriented;
        }
    }
}
