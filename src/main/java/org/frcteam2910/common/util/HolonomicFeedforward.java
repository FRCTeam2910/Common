package org.frcteam2910.common.util;

import edu.wpi.first.math.geometry.Translation2d;

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

    public Translation2d calculateFeedforward(Translation2d velocity, Translation2d acceleration) {
        // We don't use `DrivetrainFeedforwardConstants.calculateFeedforward` because we want to apply kS (the static
        // constant) proportionally based on the rest of the feedforwards.

        double forwardFeedforward = forwardConstants.getVelocityConstant() * velocity.getX();
        forwardFeedforward += forwardConstants.getAccelerationConstant() * acceleration.getX();

        double strafeFeedforward = strafeConstants.getVelocityConstant() * velocity.getY();
        strafeFeedforward += strafeConstants.getAccelerationConstant() * acceleration.getY();

        Translation2d feedforwardVector = new Translation2d(forwardFeedforward, strafeFeedforward);

        // Apply the kS constant proportionally to the forward and strafe feedforwards based on their relative
        // magnitudes
        Translation2d feedforwardUnitVector = feedforwardVector.times( 1 / feedforwardVector.getNorm());

        forwardFeedforward += Math.copySign(feedforwardUnitVector.getX() * forwardConstants.getStaticConstant(),
                forwardFeedforward);
        strafeFeedforward += Math.copySign(feedforwardUnitVector.getY() * strafeConstants.getStaticConstant(),
                strafeFeedforward);

        return new Translation2d(forwardFeedforward, strafeFeedforward);
    }

    public DrivetrainFeedforwardConstants getForwardConstants() {
        return forwardConstants;
    }

    public DrivetrainFeedforwardConstants getStrafeConstants() {
        return strafeConstants;
    }
}
