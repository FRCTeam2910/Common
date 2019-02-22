package org.frcteam2910.common.control;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import org.frcteam2910.common.util.HolonomicFeedforward;

public class HolonomicMotionProfiledTrajectoryFollower extends TrajectoryFollower<HolonomicDriveSignal> {
    private PidController forwardController;
    private PidController strafeController;
    private PidController rotationController;

    private HolonomicFeedforward feedforward;

    private boolean finished = false;

    public HolonomicMotionProfiledTrajectoryFollower(PidConstants translationConstants, PidConstants rotationConstants,
                                                     HolonomicFeedforward feedforward) {
        this.forwardController = new PidController(translationConstants);
        this.strafeController = new PidController(translationConstants);
        this.rotationController = new PidController(rotationConstants);

        this.feedforward = feedforward;
    }

    @Override
    protected HolonomicDriveSignal calculateDriveSignal(RigidTransform2 currentPose, Vector2 velocity,
                                               double rotationalVelocity, Trajectory trajectory, double time,
                                               double dt) {
        if (time > trajectory.getDuration()) {
            finished = true;
            return new HolonomicDriveSignal(Vector2.ZERO, 0.0, false);
        }

        Trajectory.Segment currentSegment = trajectory.calculateSegment(time);

        Vector2 segmentVelocity = Vector2.fromAngle(currentSegment.heading).scale(currentSegment.velocity);
        Vector2 segmentAcceleration = Vector2.fromAngle(currentSegment.heading).scale(currentSegment.acceleration);

        Vector2 feedforwardVector = feedforward.calculateFeedforward(segmentVelocity, segmentAcceleration);

        forwardController.setSetpoint(currentSegment.translation.x);
        strafeController.setSetpoint(currentSegment.translation.y);
        rotationController.setSetpoint(currentSegment.rotation.toRadians());

        return new HolonomicDriveSignal(
                new Vector2(
                        forwardController.calculate(currentPose.translation.x, dt) + feedforwardVector.x,
                        strafeController.calculate(currentPose.translation.y, dt) + feedforwardVector.y
                ),
                rotationController.calculate(currentPose.rotation.toRadians(), dt),
                true
        );
    }

    @Override
    protected boolean isFinished() {
        return finished;
    }

    @Override
    protected void reset() {
        forwardController.reset();
        strafeController.reset();
        rotationController.reset();

        finished = false;
    }
}
