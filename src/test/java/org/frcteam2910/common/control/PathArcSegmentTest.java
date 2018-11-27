package org.frcteam2910.common.control;

import org.frcteam2910.common.math.Vector2;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class PathArcSegmentTest {
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
}
