package org.frcteam2910.common.control;

import org.frcteam2910.common.math.Vector2;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class PathLineSegmentTest {
    @Test
    public void mirrorTest() {
        PathLineSegment[] originals = {
                new PathLineSegment(new Vector2(0.0, 0.0), new Vector2(0.0, 5.0)),
                new PathLineSegment(new Vector2(-1.0, 1.0), new Vector2(2.0, -2.0))
        };
        PathLineSegment[] expected = {
                new PathLineSegment(new Vector2(0.0, 0.0), new Vector2(0.0, -5.0)),
                new PathLineSegment(new Vector2(-1.0, -1.0), new Vector2(2.0, 2.0))
        };

        for (int i = 0; i < originals.length; i++) {
            PathLineSegment actual = originals[i].mirror();
            assertEquals(expected[i], actual);
        }
    }

    @Test
    public void subdivideTest() {
        PathLineSegment[] originals = new PathLineSegment[]{
                new PathLineSegment(new Vector2(0, 0), new Vector2(10, 10))
        };

        PathLineSegment[][] expected = new PathLineSegment[][]{
                {
                        new PathLineSegment(new Vector2(0, 0), new Vector2(5, 5)),
                        new PathLineSegment(new Vector2(5, 5), new Vector2(10, 10))
                }
        };

        for (int i = 0; i < originals.length; i++) {
            PathLineSegment[] actual = originals[i].subdivide();

            for (int j = 0; j < actual.length; j++) {
                assertEquals(expected[i][j], actual[j]);
            }
        }
    }
}
