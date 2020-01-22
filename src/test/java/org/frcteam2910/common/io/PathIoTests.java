package org.frcteam2910.common.io;

import org.frcteam2910.common.control.Path;
import org.frcteam2910.common.control.SplinePathBuilder;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.junit.Test;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

public class PathIoTests {
    private void assertPathsAreEqual(Path expectedPath, Path actualPath) {
        // Verify the paths are the same
        assertEquals("Path length is not correct", expectedPath.getLength(), actualPath.getLength(), MathUtils.EPSILON);


        final int iterations = 10000;
        for (int i = 0; i < iterations; i++) {
            double t = ((double) i) / (iterations - 1.0);

            Path.State expected = expectedPath.calculate(t * expectedPath.getLength());
            Path.State actual = actualPath.calculate(t * actualPath.getLength());

            assertEquals("Curvature does not match", expected.getCurvature(), actual.getCurvature(), MathUtils.EPSILON);
            assertEquals("Distance does not match", expected.getDistance(), actual.getDistance(), MathUtils.EPSILON);
            assertEquals("Heading does not match", expected.getHeading(), actual.getHeading());
            assertEquals("Position does not match", expected.getPosition(), actual.getPosition());
            assertEquals("Rotation does not match", expected.getRotation(), actual.getRotation());
        }
    }

    /**
     * Reads a path from an already existing file to ensure backwards compatibility.
     */
    @Test
    public void backwardsCompatibilityTest() throws IOException {
        Path expectedPath = new SplinePathBuilder(Vector2.ZERO, Rotation2.ZERO, Rotation2.ZERO)
                .bezier(new Vector2(5.0, 0.0), new Vector2(45.0, 50.0), new Vector2(50.0, 50.0), Rotation2.fromDegrees(90.0))
                .build();

        try (InputStream in = getClass().getResourceAsStream("example_path.json")) {
            assertNotNull("Unable to find example path", in);

            PathReader reader = new PathReader(new InputStreamReader(in, StandardCharsets.UTF_8));
            Path actualPath = reader.read();

            assertPathsAreEqual(expectedPath, actualPath);
        }
    }

    @Test
    public void readerIsCompatibleWithWriter() throws IOException {
        Path expectedPath = new SplinePathBuilder(Vector2.ZERO, Rotation2.ZERO, Rotation2.ZERO)
                .bezier(new Vector2(5.0, 0.0), new Vector2(45.0, 50.0), new Vector2(50.0, 50.0), Rotation2.fromDegrees(90.0))
                .hermite(new Vector2(0.0, 0.0), Rotation2.fromDegrees(180.0), Rotation2.fromDegrees(-90))
                .build();

        File tempFile = File.createTempFile("pathbuffer-", ".tmp");
        tempFile.deleteOnExit();

        try (PathWriter writer = new PathWriter(Files.newBufferedWriter(tempFile.toPath(), StandardCharsets.UTF_8))) {
            writer.write(expectedPath);
        }

        Path actualPath;
        try (PathReader reader = new PathReader(Files.newBufferedReader(tempFile.toPath(), StandardCharsets.UTF_8))) {
            actualPath = reader.read();
        }

        assertPathsAreEqual(expectedPath, actualPath);
    }

    @Test(expected = IOException.class)
    public void readerThrowsOnBadJson() throws IOException {
        try (StringReader reader = new StringReader("{\"this\":\"is\", \"bad\":1234}")) {
            PathReader pathReader = new PathReader(reader);
            pathReader.read();
        }
    }

    @Test(expected = IOException.class)
    public void readerThrowsOnBadInput() throws IOException {
        try (StringReader reader = new StringReader("This is not json")) {
            PathReader pathReader = new PathReader(reader);
            pathReader.read();
        }
    }

    @Test(expected = IOException.class)
    public void readerThrowsOnEmptyInput() throws IOException {
        try (StringReader reader = new StringReader("")) {
            PathReader pathReader = new PathReader(reader);
            pathReader.read();
        }
    }
}
