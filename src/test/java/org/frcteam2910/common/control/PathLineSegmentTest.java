package org.frcteam2910.common.control;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class PathLineSegmentTest {
    @Test
    public void rotationTest() {
        Path.Segment segment = new Path.Segment.Line(new RigidTransform2(Vector2.ZERO, Rotation2.ZERO),
                new RigidTransform2(new Vector2(30, 40), Rotation2.fromDegrees(90)));

        assertEquals(Rotation2.fromDegrees(0), segment.getRotationAtPercentage(0));
        assertEquals(Rotation2.fromDegrees(0), segment.getRotationAtDistance(0));

        assertEquals(Rotation2.fromDegrees(45), segment.getRotationAtPercentage(0.5));
        assertEquals(Rotation2.fromDegrees(45), segment.getRotationAtDistance(25));

        assertEquals(Rotation2.fromDegrees(90), segment.getRotationAtPercentage(1));
        assertEquals(Rotation2.fromDegrees(90), segment.getRotationAtDistance(50));
    }
}
