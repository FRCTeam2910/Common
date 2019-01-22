package org.frcteam2910.common.control;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Vector2;

import java.util.Optional;

public abstract class TrajectoryFollower<DriveSignalType> {
    private final Object trajectoryLock = new Object();

    /**
     * The trajectory that is currently being followed. Null if no trajectory is being followed.
     * <p>
     * Protected by {@link #trajectoryLock}
     */
    private Trajectory currentTrajectory = null;

    /**
     * The time that the current trajectory started to be followed. NaN if the trajectory has not been started yet.
     * <p>
     * Protected by {@link #trajectoryLock}
     */
    private double startTime = Double.NaN;

    /**
     * Calculates the drive signal required to follow the trajectory.
     *
     * @param currentPose        the current pose of the robot
     * @param velocity           the translational velocity of the robot
     * @param rotationalVelocity the rotational velocity of the robot
     * @param trajectory         the trajectory to follow
     * @param time               the amount of time that has elapsed since the current trajectory started to be followed
     * @param dt                 the amount of time that has elapsed since the update loop was last ran
     * @return the signal required to follow the trajectory
     */
    protected abstract DriveSignalType calculateDriveSignal(RigidTransform2 currentPose, Vector2 velocity,
                                                            double rotationalVelocity, Trajectory trajectory,
                                                            double time, double dt);

    /**
     * Gets if the follower is done following the path.
     *
     * @return true if the path is done
     */
    protected abstract boolean isFinished();

    /**
     * Resets the follower's internal state. This is called when a new trajectory is started.
     */
    protected abstract void reset();

    /**
     * Cancels the currently running trajectory.
     */
    public final void cancel() {
        synchronized (trajectoryLock) {
            currentTrajectory = null;
        }
    }

    public final void follow(Trajectory trajectory) {
        synchronized (trajectoryLock) {
            currentTrajectory = trajectory;
            startTime = Double.NaN;
        }
    }

    /**
     * Gets the current trajectory that is being followed if any.
     *
     * @return the current trajectory being followed
     */
    public final Optional<Trajectory> getCurrentTrajectory() {
        synchronized (trajectoryLock) {
            return Optional.ofNullable(currentTrajectory);
        }
    }

    /**
     * Updates the follower and returns the calculated drive signal that should be applied to the robot in order to
     * follow the current trajectory.
     *
     * @param currentPose        the current pose of the robot
     * @param velocity           the translational velocity of the robot
     * @param rotationalVelocity the rotational velocity of the robot
     * @param time               the current time
     * @param dt                 the time since update was last called
     * @return the drive signal required to follow the current path if any
     */
    public final Optional<DriveSignalType> update(RigidTransform2 currentPose, Vector2 velocity,
                                                  double rotationalVelocity, double time, double dt) {
        Trajectory trajectory;
        double timeSinceStart;

        synchronized (trajectoryLock) {
            // Return empty if no trajectory is being followed
            if (currentTrajectory == null) {
                return Optional.empty();
            }

            // If the trajectory has not been started, update the start time and reset the follower state
            if (Double.isNaN(startTime)) {
                startTime = time;
                reset();
            } else if (isFinished()) {
                currentTrajectory = null;
                return Optional.empty();
            }

            trajectory = currentTrajectory;
            timeSinceStart = time - startTime;
        }

        DriveSignalType signal = calculateDriveSignal(currentPose, velocity, rotationalVelocity, trajectory,
                timeSinceStart, dt);

        return Optional.of(signal);
    }
}
