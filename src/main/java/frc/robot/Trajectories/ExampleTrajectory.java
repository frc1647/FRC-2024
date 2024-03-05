package frc.robot.Trajectories;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

class ExampleTrajectory {

    Trajectory trajectory;

        public void generateTrajectory() {

        // 2018 cross scale auto waypoints.
        var sideStart = new Pose2d(0, 0,
            Rotation2d.fromDegrees(-180));
        var crossScale = new Pose2d(.5, 0,
            Rotation2d.fromDegrees(-180));

        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(4.5, 7));
        interiorWaypoints.add(new Translation2d(7, 5));

        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        config.setReversed(true);

        trajectory = TrajectoryGenerator.generateTrajectory(
            sideStart,
            interiorWaypoints,
            crossScale,
            config);
    }

    public Trajectory gTrajectory(){
        return trajectory;
    }
  }
