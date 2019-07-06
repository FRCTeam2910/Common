package org.frcteam2910.common.motion;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class TrapezoidalMotionProfileTest {
    private static final double ALLOWABLE_POSITION_ERROR = 1.0e-9;
    private static final double ALLOWABLE_VELOCITY_ERROR = 1.0e-9;

    private void verifyStartEnd(TrapezoidalMotionProfile profile) {
        MotionProfile.State profileStart = profile.calculate(0.0);
        MotionProfile.State profileEnd = profile.calculate(profile.getDuration());

        assertEquals("Unexpected profile start position", profile.getStart().position, profileStart.position, ALLOWABLE_POSITION_ERROR);
        assertEquals("Unexpected profile start velocity", profile.getStart().velocity, profileStart.velocity, ALLOWABLE_VELOCITY_ERROR);
        assertEquals("Unexpected profile end position", profile.getEnd().position, profileEnd.position, ALLOWABLE_POSITION_ERROR);
        assertEquals("Unexpected profile end velocity", profile.getEnd().velocity, profileEnd.velocity, ALLOWABLE_VELOCITY_ERROR);
    }

    @Test
    public void forwardStartEndTest() {
        TrapezoidalMotionProfile profile = new TrapezoidalMotionProfile(
                new MotionProfile.Goal(0.0, 0.0),
                new MotionProfile.Goal(10.0, 0.0),
                new MotionProfile.Constraints(1.0, 1.0)
        );
        verifyStartEnd(profile);
    }

    @Test
    public void forwardNonZeroStartEndTest() {
        TrapezoidalMotionProfile profile = new TrapezoidalMotionProfile(
                new MotionProfile.Goal(-1.0, 2.5),
                new MotionProfile.Goal(10.0, 0.5),
                new MotionProfile.Constraints(5.0, 2.0)
        );
        verifyStartEnd(profile);
    }

    @Test
    public void reverseStartEndTest() {
        TrapezoidalMotionProfile profile = new TrapezoidalMotionProfile(
                new MotionProfile.Goal(10.0, 0.0),
                new MotionProfile.Goal(0.0, 0.0),
                new MotionProfile.Constraints(1.0, 1.0)
        );
        verifyStartEnd(profile);
    }

    @Test
    public void reverseNonZeroStartEndTest() {
        TrapezoidalMotionProfile profile = new TrapezoidalMotionProfile(
                new MotionProfile.Goal(10.0, 2.5),
                new MotionProfile.Goal(-2.3, -2.0),
                new MotionProfile.Constraints(5.0, 2.0)
        );
        verifyStartEnd(profile);
    }
}
