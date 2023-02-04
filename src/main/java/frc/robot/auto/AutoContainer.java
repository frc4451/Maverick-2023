/*
AutoContainer holds public static methods for autonomous routines
Each static method will be one entire autonomous routine called in robot.autonomousPeriodic state
The autonomous mode is selected from the driver station dashboard through a sendable chooser
 */
package frc.robot.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
// import edu.wpi.first.math.trajectory.Trajectory;
// import frc.robot.trajectories.TrajectoryContainer;
import frc.robot.trajectories.TrajectoryContainer;

public class AutoContainer {
    private static final Timer autoTimer = new Timer();

    private static int autoStep = 0;

    public static void resetAutoStep() {
        AutoContainer.autoStep = 0;
    }

    private static void resetTimer() {
        autoTimer.reset();
        autoTimer.start();
    }

    /**
     * Sets driveTrain pose to trajectory's inital pose
     * 
     * @param trajectory Trajectory to get inital pose from
     */
    private static void setNavigationToTrajectoryStart(Trajectory trajectory) {
        RobotContainer.driveTrain.resetNavigation(trajectory.getInitialPose());
    }

    private static void doTrajectory(Trajectory trajectory, int nextAutoStep) {
        if (autoTimer.get() < trajectory.getTotalTimeSeconds()) {
            // Get the desired pose from the trajectory.
            Trajectory.State desiredPose = trajectory.sample(autoTimer.get());

            // Get the reference chassis speeds from the Ramsete controller.
            ChassisSpeeds refChassisSpeeds = RobotContainer.driveTrain.ramseteCalculate(desiredPose);

            // Set the linear and angular speeds.
            RobotContainer.driveTrain.kinematicDrive(
                    refChassisSpeeds.vxMetersPerSecond,
                    refChassisSpeeds.omegaRadiansPerSecond);
        } else {
            RobotContainer.driveTrain.kinematicDrive(0, 0);
            autoStep = nextAutoStep;
        }
    }

    public static void auto1() {
        switch (autoStep) {
            case 0:
                // Reset the drivetrain's odometry to the starting pose of the trajectory.
                setNavigationToTrajectoryStart(TrajectoryContainer.trajectory1_1);
                resetTimer();
                autoStep = 1;
                break;
            case 1:
                doTrajectory(TrajectoryContainer.trajectory1_1, 2);
                break;
            case 2:
                resetTimer();
                autoStep = 3;
            case 3:
                doTrajectory(TrajectoryContainer.trajectory1_2, -1);
                break;
        }

    }

    public static void auto2() {
        switch (autoStep) {
            case 0:
                // Reset the drivetrain's odometry to the starting pose of the trajectory.
                setNavigationToTrajectoryStart(TrajectoryContainer.trajectory3);
                resetTimer();
                autoStep = 1;
                break;
            case 1:
                doTrajectory(TrajectoryContainer.trajectory3, -1);
                break;
        }
    }

    public static void auto3() {
        switch (autoStep) {
            case 0:
                // Reset the drivetrain's odometry to the starting pose of the trajectory.
                setNavigationToTrajectoryStart(TrajectoryContainer.charge);
                resetTimer();
                autoStep = 1;
                break;
            case 1:
                doTrajectory(TrajectoryContainer.charge, 2);
                break;
            case 2:
                // Cool balancing stuff here
                RobotContainer.driveTrain.balanceChargeStation();
                // doTrajectory(TrajectoryContainer.charge, -1);
                break;
        }
    }
}
