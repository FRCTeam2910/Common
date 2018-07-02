package org.frcteam2910.common.math;

import org.junit.Test;

import static org.frcteam2910.common.math.MathUtils.EPSILON;
import static org.junit.Assert.assertEquals;

public class Vector2Test {
    @Test
    public void angle() {
        Vector2 vector = new Vector2(0, 1);
        assertEquals(vector.angle, Math.PI / 2, EPSILON);

        vector = new Vector2(0, -1);
        assertEquals(vector.angle, -Math.PI / 2, EPSILON);

        vector = new Vector2(1, 0);
        assertEquals(vector.angle, 0, EPSILON);

        vector = new Vector2(-1, 0);
        assertEquals(vector.angle, Math.PI, EPSILON);

        vector = new Vector2(1, 1);
        assertEquals(vector.angle, Math.PI / 4, EPSILON);
    }

    @Test
    public void length() {
        Vector2 vector = new Vector2(1, 1);
        assertEquals(vector.length, Math.sqrt(2), EPSILON);

        vector = new Vector2(-3, 4);
        assertEquals(vector.length, 5, EPSILON);
    }

    @Test
    public void addVector() {
        Vector2 a = new Vector2(0, 2);
        Vector2 b = new Vector2(2, 2);
        assertEquals(a.add(b), new Vector2(2, 4));

        a = new Vector2(-5, 2.5);
        b = new Vector2(10, 2.7);
        assertEquals(a.add(b), new Vector2(5, 5.2));
    }

    @Test
    public void multiplyScalar() {
        Vector2 vector = new Vector2();
        double scalar = 1;
        assertEquals(vector.multiply(scalar), new Vector2(0, 0));

        vector = new Vector2(-5, 2.2);
        scalar = 10;
        assertEquals(vector.multiply(scalar), new Vector2(-50, 22));
    }

    @Test
    public void rotateBy() {
        Vector2 vector = new Vector2(5, 0);
        assertEquals(vector.rotateBy(Math.toRadians(90)), new Vector2(0, 5));

        vector = new Vector2(5, 5);
        assertEquals(vector.rotateBy(Math.toRadians(-45)), new Vector2(Math.hypot(5, 5), 0));
    }
}
