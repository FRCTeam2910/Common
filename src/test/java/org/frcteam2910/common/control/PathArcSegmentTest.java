package org.frcteam2910.common.control;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class PathArcSegmentTest {
    @Test
    public void mirrorTest() {
        PathArcSegment[] originals = {
                new PathArcSegment(new Vector2(-5.0, 0.0), new Vector2(0.0, 5.0), new Vector2(-5.0, 5.0)),
                new PathArcSegment(new Vector2(1.0, 0.0), new Vector2(0.0, 1.0), new Vector2(0.0, 0.0))
        };
        PathArcSegment[] expected = {
                new PathArcSegment(new Vector2(-5.0, 0.0), new Vector2(0.0, -5.0), new Vector2(-5.0, -5.0)),
                new PathArcSegment(new Vector2(1.0, 0.0), new Vector2(0.0, -1.0), new Vector2(0.0, -0.0))
        };

        for (int i = 0; i < originals.length; i++) {
            PathArcSegment actual = originals[i].mirror();
            assertEquals(expected[i], actual);
        }
    }

    @Test
    public void subdivideTest() {
        PathArcSegment[] originals = new PathArcSegment[]{
                new PathArcSegment(new Vector2(1, 0), new Vector2(0, 1), Vector2.ZERO),
                new PathArcSegment(new Vector2(Math.sqrt(2.0), Math.sqrt(2.0)), new Vector2(-Math.sqrt(2.0), Math.sqrt(2.0)), Vector2.ZERO)
        };

        PathArcSegment[][] expected = new PathArcSegment[][]{
                {
                        new PathArcSegment(new Vector2(1, 0), new Vector2(Math.sqrt(2.0) / 2, Math.sqrt(2.0) / 2), Vector2.ZERO),
                        new PathArcSegment(new Vector2(Math.sqrt(2.0) / 2, Math.sqrt(2.0) / 2), new Vector2(0, 1), Vector2.ZERO)
                },
                {
                        new PathArcSegment(new Vector2(Math.sqrt(2), Math.sqrt(2)), new Vector2(0, 2), Vector2.ZERO),
                        new PathArcSegment(new Vector2(0, 2), new Vector2(-Math.sqrt(2), Math.sqrt(2)), Vector2.ZERO)
                }
        };

        for (int i = 0; i < originals.length; i++) {
            PathArcSegment[] actual = originals[i].subdivide();

            for (int j = 0; j < actual.length; j++) {
                assertEquals(expected[i][j], actual[j]);
            }
        }
    }

    @Test
    public void getHeadingTest() {
        PathArcSegment segment = new PathArcSegment(new Vector2(1.0, 0.0), new Vector2(0.0, 1.0), Vector2.ZERO);
        assertEquals(Rotation2.fromDegrees(90.0), segment.getHeadingAtPercentage(0.0));
        assertEquals(Rotation2.fromDegrees(135.0), segment.getHeadingAtPercentage(0.5));
        assertEquals(Rotation2.fromDegrees(180.0), segment.getHeadingAtPercentage(1.0));

        segment = new PathArcSegment(new Vector2(0.0, 1.0), new Vector2(1.0, 0.0), Vector2.ZERO);
        assertEquals(Rotation2.fromDegrees(0.0), segment.getHeadingAtPercentage(0.0));
        assertEquals(Rotation2.fromDegrees(-45.0), segment.getHeadingAtPercentage(0.5));
        assertEquals(Rotation2.fromDegrees(-90.0), segment.getHeadingAtPercentage(1.0));
    }
}
