package org.frcteam2910.common.util;

public class DrivetrainFeedforwardConstants {
    private final double velocityConstant;
    private final double accelerationConstant;
    private final double staticConstant;

    public DrivetrainFeedforwardConstants(double velocityConstant, double accelerationConstant, double staticConstant) {
        this.velocityConstant = velocityConstant;
        this.accelerationConstant = accelerationConstant;
        this.staticConstant = staticConstant;
    }

    public double calculateFeedforward(double velocity, double acceleration) {
        double feedforward = velocityConstant * velocity;
        feedforward += accelerationConstant * acceleration;

        feedforward += Math.copySign(staticConstant, feedforward);

        return feedforward;
    }

    public double getVelocityConstant() {
        return velocityConstant;
    }

    public double getAccelerationConstant() {
        return accelerationConstant;
    }

    public double getStaticConstant() {
        return staticConstant;
    }
}
