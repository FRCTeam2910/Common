package org.frcteam2910.common.control;

import org.frcteam2910.common.motion.MotionProfile;

import java.util.Optional;

public class MotionProfileFollower {
    private final Object profileLock = new Object();
    private MotionProfile currentMotionProfile = null;
    private double startTime = Double.NaN;

    private final PidController controller;
    private final double kV;
    private final double kA;

    private MotionProfile.State lastState = null;

    public MotionProfileFollower(PidController controller, double kV, double kA) {
        this.controller = controller;
        this.kV = kV;
        this.kA = kA;
    }

    public double update(double currentPosition,
                         double time,
                         double dt) {
        MotionProfile profile;
        double timeSinceStart;

        synchronized (profileLock) {
            if (currentMotionProfile == null) {
                return 0.0;
            }

            if (Double.isNaN(startTime)) {
                startTime = time;
            }

            profile = currentMotionProfile;
            timeSinceStart = time - startTime;
        }

        MotionProfile.State state = profile.calculate(timeSinceStart);
        synchronized (profileLock) {
            lastState = state;
        }

        controller.setSetpoint(state.position);
        return controller.calculate(currentPosition, dt) + kV * state.velocity + kA * state.acceleration;
    }

    public void follow(MotionProfile motionProfile) {
        synchronized (profileLock) {
            currentMotionProfile = motionProfile;
            startTime = Double.NaN;
        }
    }

    public MotionProfile getCurrentMotionProfile() {
        synchronized (profileLock) {
            return currentMotionProfile;
        }
    }

    public Optional<MotionProfile.State> getLastState() {
        synchronized (profileLock) {
            return Optional.ofNullable(lastState);
        }
    }

    public void cancel() {
        synchronized (profileLock) {
            currentMotionProfile = null;
        }
    }
}
