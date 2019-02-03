package org.frcteam2910.common.util;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Vector2;
import org.junit.Test;

import static org.junit.Assert.*;

public class HolonomicDriveSignalTest {
    @Test
    public void getTranslationTest() {
        HolonomicDriveSignal signal = new HolonomicDriveSignal(Vector2.ZERO, 0.0, false);
        assertEquals(Vector2.ZERO, signal.getTranslation());

        signal = new HolonomicDriveSignal(new Vector2(2.5, -2.4), 0.0, false);
        assertEquals(new Vector2(2.5, -2.4), signal.getTranslation());
    }

    @Test
    public void getRotationTest() {
        HolonomicDriveSignal signal = new HolonomicDriveSignal(Vector2.ZERO, 0.0, false);
        assertEquals(0.0, signal.getRotation(), MathUtils.EPSILON);

        signal = new HolonomicDriveSignal(Vector2.ZERO, 1.2, false);
        assertEquals(1.2, signal.getRotation(), MathUtils.EPSILON);
    }

    @Test
    public void isFieldOrientedTest() {
        HolonomicDriveSignal signal = new HolonomicDriveSignal(Vector2.ZERO, 0.0, false);
        assertFalse(signal.isFieldOriented());

        signal = new HolonomicDriveSignal(Vector2.ZERO, 0.0, true);
        assertTrue(signal.isFieldOriented());
    }
}
