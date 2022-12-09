package org.frcteam2910.common.util;

import edu.wpi.first.math.geometry.Translation2d;
import org.frcteam2910.common.math.MathUtils;
import org.junit.Test;

import static org.junit.Assert.*;

public class HolonomicDriveSignalTest {
    @Test
    public void getTranslationTest() {
        HolonomicDriveSignal signal = new HolonomicDriveSignal(new Translation2d(), 0.0, false);
        assertEquals(new Translation2d(), signal.getTranslation());

        signal = new HolonomicDriveSignal(new Translation2d(2.5, -2.4), 0.0, false);
        assertEquals(new Translation2d(2.5, -2.4), signal.getTranslation());
    }

    @Test
    public void getRotationTest() {
        HolonomicDriveSignal signal = new HolonomicDriveSignal(new Translation2d(), 0.0, false);
        assertEquals(0.0, signal.getRotation(), MathUtils.EPSILON);

        signal = new HolonomicDriveSignal(new Translation2d(), 1.2, false);
        assertEquals(1.2, signal.getRotation(), MathUtils.EPSILON);
    }

    @Test
    public void isFieldOrientedTest() {
        HolonomicDriveSignal signal = new HolonomicDriveSignal(new Translation2d(), 0.0, false);
        assertFalse(signal.isFieldOriented());

        signal = new HolonomicDriveSignal(new Translation2d(), 0.0, true);
        assertTrue(signal.isFieldOriented());
    }
}
