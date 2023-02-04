// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trajectories;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class TrajectoryContainer {
    public static final Trajectory trajectory1_1 = genTrajectory1();
    public static final Trajectory trajectory1_2 = genTrajectory2();
    public static final Trajectory trajectory3 = genFromPathWeaver("testz.wpilib.json");
    public static final Trajectory charge = genFromPathWeaver("charge.wpilib.json");

    private static Trajectory genTrajectory1() {
        Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        ArrayList<Translation2d> waypoints = new ArrayList<Translation2d>();
        // Use waypoints.add to add waypoints
        waypoints.add(new Translation2d(1.0, 0.2));
        Pose2d endPose = new Pose2d(3, 2, Rotation2d.fromDegrees(90));
        TrajectoryConfig config = new TrajectoryConfig(1.5, 1.0);
        config.setStartVelocity(0.0);
        config.setEndVelocity(0.0);
        config.setReversed(false);
        Trajectory path = TrajectoryGenerator.generateTrajectory(
                startPose,
                waypoints,
                endPose,
                config);
        return path;
    }

    private static Trajectory genTrajectory2() {
        Pose2d startPose = new Pose2d(3, 2, Rotation2d.fromDegrees(90));
        ArrayList<Translation2d> waypoints = new ArrayList<Translation2d>();
        // Use waypoints.add to add waypoints
        waypoints.add(new Translation2d(1.0, -0.3));
        Pose2d endPose = new Pose2d(0, -0.5, Rotation2d.fromDegrees(0));
        TrajectoryConfig config = new TrajectoryConfig(1.5, 1.0);
        config.setStartVelocity(0.0);
        config.setEndVelocity(0.0);
        config.setReversed(true);
        Trajectory path = TrajectoryGenerator.generateTrajectory(
                startPose,
                waypoints,
                endPose,
                config);
        return path;
    }

    private static Trajectory genFromPathWeaver(String jsonPath) {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("output/" + jsonPath);
            return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory " + jsonPath + "; Loading empty trajectory, ",
                    ex.getStackTrace());
            return new Trajectory();
        }
    }
}