package org.frcteam2910.common.math;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class Rotation2Test {
    @Test
    public void fromDegrees() {
        assertEquals(Rotation2.fromDegrees(90), new Rotation2(0, 1, false));
        assertEquals(Rotation2.fromDegrees(-180), new Rotation2(-1, 0, false));
        assertEquals(Rotation2.fromDegrees(45), new Rotation2(1, 1, true));
    }

    @Test
    public void fromRadians() {
        assertEquals(Rotation2.fromRadians(Math.PI), new Rotation2(-1, 0, false));
        assertEquals(Rotation2.fromRadians(2 * Math.PI), new Rotation2(1, 0, false));
        assertEquals(Rotation2.fromRadians(-3 * Math.PI / 4), new Rotation2(-1, -1, true));
    }

    @Test
    public void toDegrees() {
        assertEquals(Rotation2.fromDegrees(23.4).toDegrees(), 23.4, MathUtils.EPSILON);
        assertEquals(Rotation2.fromDegrees(-5).toDegrees(), 355, MathUtils.EPSILON);
        assertEquals(Rotation2.fromDegrees(365).toDegrees(), 5, MathUtils.EPSILON);
    }

    @Test
    public void toRadians() {
        assertEquals(Rotation2.fromRadians(Math.PI).toRadians(), Math.PI, MathUtils.EPSILON);
        assertEquals(Rotation2.fromRadians(3 * Math.PI / 4).toRadians(), 3 * Math.PI / 4, MathUtils.EPSILON);
        assertEquals(Rotation2.fromRadians(7 * Math.PI / 3).toRadians(), Math.PI / 3, MathUtils.EPSILON);
    }

    @Test
    public void rotateBy() {
        assertEquals(Rotation2.fromDegrees(50), Rotation2.fromDegrees(20).rotateBy(Rotation2.fromDegrees(30)));
        assertEquals(Rotation2.fromDegrees(180), Rotation2.fromDegrees(-50).rotateBy(Rotation2.fromDegrees(230)));
        assertEquals(Rotation2.fromDegrees(180), Rotation2.fromDegrees(0).rotateBy(Rotation2.fromDegrees(180)));
    }

    @Test
    public void normal() {
        assertEquals(Rotation2.fromDegrees(0).normal(), Rotation2.fromDegrees(90));
        assertEquals(Rotation2.fromDegrees(45).normal(), Rotation2.fromDegrees(135));
        assertEquals(Rotation2.fromRadians(-Math.PI / 2).normal(), Rotation2.fromDegrees(0));
    }

    @Test
    public void inverse() {
        assertEquals(Rotation2.fromRadians(Math.PI).inverse(), Rotation2.fromRadians(Math.PI));
        assertEquals(Rotation2.fromDegrees(52.3).inverse(), Rotation2.fromDegrees(-52.3));
        assertEquals(Rotation2.fromRadians(Math.PI / 2).inverse(), Rotation2.fromRadians(-Math.PI / 2));
    }

    @Test
    public void isParallel() {
        assertTrue(Rotation2.fromRadians(Math.PI).isParallel(Rotation2.fromDegrees(180)));
        assertTrue(Rotation2.fromDegrees(45).isParallel(Rotation2.fromDegrees(225)));
        assertTrue(Rotation2.fromRadians(6 * Math.PI / 11).isParallel(Rotation2.fromRadians(17 * Math.PI / 11)));
    }

    @Test
    public void interpolate() {
        assertEquals(Rotation2.fromDegrees(45), Rotation2.fromDegrees(0).interpolate(Rotation2.fromDegrees(90), 0.5));
        assertEquals(Rotation2.fromDegrees(0), Rotation2.fromDegrees(-45).interpolate(Rotation2.fromDegrees(45), 0.5));
        assertEquals(Rotation2.ZERO, Rotation2.ZERO.interpolate(Rotation2.ZERO, 0.25));
    }

    @Test
    public void equals() {
        assertEquals(Rotation2.fromRadians(0), Rotation2.fromDegrees(0));
        assertEquals(Rotation2.fromDegrees(90), Rotation2.fromRadians(Math.PI / 2));
    }
}
