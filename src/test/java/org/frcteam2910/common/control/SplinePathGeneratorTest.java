package org.frcteam2910.common.control;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.junit.Ignore;
import org.junit.Test;

import java.io.BufferedOutputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class SplinePathGeneratorTest {
    private static final double ALLOWABLE_HEADING_ERROR = Math.toRadians(0.05);
    private static final double ALLOWABLE_ROTATION_ERROR = Math.toRadians(0.05);
    private static final double ALLOWABLE_POSITION_ERROR = 0.05;

    private static final double WRITE_SEARCH = 0.1;

    @Test
    public void verifyStartAndEndTest() {
        SplinePathGenerator generator = new SplinePathGenerator();
        Path path = generator.generate(new Waypoint(Vector2.ZERO, Rotation2.ZERO),
                new Waypoint(new Vector2(12.0, 12.0), Rotation2.ZERO));


        assertTrue(String.format("Starting heading (%s) does not match the desired heading (%s)",
                path.getHeadingAtDistance(0.0), Rotation2.ZERO),
                path.getHeadingAtDistance(0.0).equals(Rotation2.ZERO, ALLOWABLE_HEADING_ERROR));
        assertEquals("Starting X position does not match desired X position",
                0.0, path.getPositionAtDistance(0.0).x, ALLOWABLE_POSITION_ERROR);
        assertEquals("Starting Y position does not match the desired Y position",
                0.0, path.getPositionAtDistance(0.0).y, ALLOWABLE_POSITION_ERROR);
        assertTrue(String.format("Starting rotation (%s) does not match the desired rotation (%s)",
                path.getRotationAtDistance(0.0), Rotation2.ZERO),
                path.getRotationAtDistance(0.0).equals(Rotation2.ZERO, ALLOWABLE_ROTATION_ERROR));

        assertTrue(String.format("Ending heading (%s) does not match the desired heading (%s)",
                path.getHeadingAtDistance(path.getLength()), Rotation2.ZERO),
                path.getHeadingAtDistance(path.getLength()).equals(Rotation2.ZERO, ALLOWABLE_HEADING_ERROR));
        assertEquals("Ending X position does not match desired X position",
                12.0, path.getPositionAtDistance(path.getLength()).x, ALLOWABLE_POSITION_ERROR);
        assertEquals("Ending Y position does not match the desired Y position",
                12.0, path.getPositionAtDistance(path.getLength()).y, ALLOWABLE_POSITION_ERROR);
        assertTrue(String.format("Ending rotation (%s) does not match the desired rotation (%s)",
                path.getRotationAtDistance(path.getLength()), Rotation2.ZERO),
                path.getRotationAtDistance(path.getLength()).equals(Rotation2.ZERO, ALLOWABLE_ROTATION_ERROR));
    }

    @Ignore
    @Test
    public void writeCsv() {
        SplinePathGenerator generator = new SplinePathGenerator();
        Path path = generator.generate(new Waypoint(Vector2.ZERO, Rotation2.ZERO),
                new Waypoint(new Vector2(12.0, 12.0), Rotation2.ZERO));

        try (PrintStream out = new PrintStream(new BufferedOutputStream(new FileOutputStream("path.csv")))) {
            out.printf("distance,x,y,heading,rotation%n");

            for (int i = 0; i < Math.ceil(path.getLength() / WRITE_SEARCH); i++) {
                double distance = i * WRITE_SEARCH;
                Vector2 position = path.getPositionAtDistance(distance);
                Rotation2 heading = path.getHeadingAtDistance(distance);
                Rotation2 rotation = path.getRotationAtDistance(distance);
                out.printf("%f,%f,%f,%f,%f%n", distance, position.x, position.y, heading.toDegrees(),
                        rotation.toDegrees());
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }
}
