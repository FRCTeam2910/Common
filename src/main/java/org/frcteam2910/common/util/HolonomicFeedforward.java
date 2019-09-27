package org.frcteam2910.common.util;

import org.frcteam2910.common.math.Vector2;

public class HolonomicFeedforward {
    private final DrivetrainFeedforwardConstants forwardConstants;
    private final DrivetrainFeedforwardConstants strafeConstants;

    public HolonomicFeedforward(DrivetrainFeedforwardConstants forwardConstants,
                                DrivetrainFeedforwardConstants strafeConstants) {
        this.forwardConstants = forwardConstants;
        this.strafeConstants = strafeConstants;
    }

    public HolonomicFeedforward(DrivetrainFeedforwardConstants translationConstants) {
        this(translationConstants, translationConstants);
    }

    public Vector2 calculateFeedforward(Vector2 velocity, Vector2 acceleration) {
        // We don't use `DrivetrainFeedforwardConstants.calculateFeedforward` because we want to apply kS (the static
        // constant) proportionally based on the rest of the feedforwards.

        double forwardFeedforward = forwardConstants.getVelocityConstant() * velocity.x;
        forwardFeedforward += forwardConstants.getAccelerationConstant() * acceleration.x;

        double strafeFeedforward = strafeConstants.getVelocityConstant() * velocity.y;
        strafeFeedforward += strafeConstants.getAccelerationConstant() * acceleration.y;

        Vector2 feedforwardVector = new Vector2(forwardFeedforward, strafeFeedforward);

        // Apply the kS constant proportionally to the forward and strafe feedforwards based on their relative
        // magnitudes
        Vector2 feedforwardUnitVector = feedforwardVector.normal();
        forwardFeedforward += Math.copySign(feedforwardUnitVector.x * forwardConstants.getStaticConstant(),
                forwardFeedforward);
        strafeFeedforward += Math.copySign(feedforwardUnitVector.y * strafeConstants.getStaticConstant(),
                strafeFeedforward);

        return new Vector2(forwardFeedforward, strafeFeedforward);
    }

    public DrivetrainFeedforwardConstants getForwardConstants() {
        return forwardConstants;
    }

    public DrivetrainFeedforwardConstants getStrafeConstants() {
        return strafeConstants;
    }
}
