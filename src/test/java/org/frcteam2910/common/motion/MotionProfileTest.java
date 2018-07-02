package org.frcteam2910.common.motion;

import org.frcteam2910.common.Constants;
import org.junit.Test;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class MotionProfileTest {
	public static final MotionProfile.Constraints CONSTRAINTS = new MotionProfile.Constraints(10, 5);

	@Test
	public void genCsv() {
		MotionProfile profile = new TrapezoidalMotionProfile(new MotionProfile.Goal(0, 0),
				new MotionProfile.Goal(15, 0), CONSTRAINTS);

		try (PrintWriter writer = new PrintWriter(new FileWriter("profile.csv"))) {
			writer.format("%s,%s,%s,%s%n", "time", "position", "velocity", "acceleration");
			for (double t = 0; t <= profile.getDuration(); t += 5 / Constants.MILLISECONDS) {
				MotionProfile.State state = profile.calculate(t);
				writer.format("%f,%f,%f,%f%n", t, state.position, state.velocity, state.acceleration);
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
