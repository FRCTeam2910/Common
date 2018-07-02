package org.frcteam2910.common.control;

import com.team254.lib.util.math.Translation2d;
import org.junit.Test;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class PathTest {
	@Test
	public void writePositionCSV() {
		Path path = PathBuilder.build(
				new Waypoint(new Translation2d(0, 0), 0),
				new Waypoint(new Translation2d(5, 100), 30),
				new Waypoint(new Translation2d(-100, 80), 10),
				new Waypoint(new Translation2d(-80, 110), 20),
				new Waypoint(new Translation2d(-50, 200), 0)
		);

		try (PrintWriter writer = new PrintWriter(new FileWriter("path.csv"))) {
			writer.format("%s,%s,%s%n", "position", "x", "y");
			for (double t = 0; t <= 1; t += 1e-3) {
				double distance = t * path.getLength();
				Translation2d position = path.getPositionAtDistance(distance);
				writer.format("%f,%f,%f%n", distance, position.x(), position.y());
				writer.flush();
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
