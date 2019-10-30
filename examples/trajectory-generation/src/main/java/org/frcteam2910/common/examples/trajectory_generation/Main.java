package org.frcteam2910.common.examples.trajectory_generation;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVPrinter;
import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.io.FileWriter;
import java.io.IOException;

public class Main {
    public static void main(String[] args) {
        // First we have to generate our path. We will use the SplinePathBuilder to generate a path using splines.
        Path path = new SplinePathBuilder(Vector2.ZERO, Rotation2.ZERO, Rotation2.ZERO)
                // When using hermite splines we must specify a position and a heading. We can also optionally specify
                // a rotation.
                .hermite(new Vector2(100.0, 100.0), Rotation2.ZERO, Rotation2.fromDegrees(90.0))
                .hermite(new Vector2(50.0, 50.0), Rotation2.fromDegrees(180.0), Rotation2.ZERO)
                // Once we've added all the splines we can then build the path.
                .build();

        // Once we have our path we need to then specify some constraints for our trajectory.
        TrajectoryConstraint[] constraints = {
                // Lets specify a maximum acceleration of 10.0 units/s^2
                new MaxAccelerationConstraint(10.0),
                // And lets have a maximum velocity of 12.0 units/s
                new MaxVelocityConstraint(12.0)
        };

        // Now that we have both our path and our constraints we can create a trajectory.
        // When creating a trajectory we pass in our path and our constraints.
        // We also have to pass in a third parameter called sample distance. This sample distance
        // determines how often the trajectory makes sure that the velocity and acceleration are within
        // the limits determined by the constraints. Smaller values will create a smoother and more accurate path
        // but they will take much longer to generate.
        Trajectory trajectory = new Trajectory(path, constraints, 1.0e-2);

        // Now that we have our trajectory lets evaluate it at a 5ms sample period and save it to a csv.
        try (CSVPrinter printer = new CSVPrinter(new FileWriter("trajectory.csv"), CSVFormat.RFC4180)) {
            // Print out the header for the csv file
            printer.printRecord(
                    "time",
                    "distance",
                    "x",
                    "y",
                    "heading",
                    "rotation",
                    "curvature",
                    "velocity",
                    "acceleration"
            );

            int samples = (int) Math.ceil(trajectory.getDuration() / 5.0e-3);
            for (int i = 0; i <= samples; i++) {
                // Calculate the trajectory state at the given time
                Trajectory.State state = trajectory.calculate(i * 5.0e-3);

                // Write the trajectory state to the csv file
                printer.printRecord(
                        i * 5.0e-3,
                        state.getPathState().getDistance(),
                        state.getPathState().getPosition().x,
                        state.getPathState().getPosition().y,
                        state.getPathState().getHeading().toDegrees(),
                        state.getPathState().getRotation().toDegrees(),
                        state.getPathState().getCurvature(),
                        state.getVelocity(),
                        state.getAcceleration()
                );
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
