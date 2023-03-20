/*
 * AutoContainer holds public static methods for autonomous routines
 * Each static method will be one entire autonomous routine called in Robot.autonomousPeriodic()
 * The autonomous mode is selected from the dashboard through a SendableChooser
 */
package frc.robot.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SubIntakeModes;

public class AutoContainer {
    private static final Timer autoTimer = new Timer();
    private static final Timer miniTimer = new Timer();

    private static int autoStep = 0;

    public static void resetAutoStep() {
        AutoContainer.autoStep = 0;
    }

    private static void resetAutoTimer() {
        autoTimer.reset();
        autoTimer.start();
    }

    private static void resetMiniTimer() {
        miniTimer.reset();
        miniTimer.start();
    }

    private static void setAutoStep(int value) {
        autoStep = value;
        resetAutoTimer();
        resetMiniTimer();
    }

    private static void incAutoStep(int value) {
        setAutoStep(autoStep + value);
    }

    private static void incAutoStep() {
        incAutoStep(1);
    }

    /**
     * Sets driveTrain pose to trajectory's inital pose
     * 
     * @param trajectory Trajectory to get inital pose from
     */
    private static void setNavigationToTrajectoryStart(Trajectory trajectory) {
        RobotContainer.driveTrain.resetNavigation(trajectory.getInitialPose());
    }

    private static void doTrajectory(PathPlannerTrajectory trajectory, int nextAutoStep) {
        if (autoTimer.get() < trajectory.getTotalTimeSeconds()) {
            // Get the desired pose from the trajectory.
            PathPlannerState desiredPose = (PathPlannerState) trajectory.sample(autoTimer.get());

            // Get the reference chassis speeds from the Ramsete controller.
            ChassisSpeeds refChassisSpeeds = RobotContainer.driveTrain.ramseteCalculate(desiredPose);

            // Set the linear and angular speeds.
            RobotContainer.driveTrain.drive(
                    refChassisSpeeds.vxMetersPerSecond,
                    refChassisSpeeds.omegaRadiansPerSecond);
        } else {
            RobotContainer.driveTrain.drive(0, 0);
            incAutoStep();
        }
    }

    private static void doTrajectory(PathPlannerTrajectory trajectory) {
        doTrajectory(trajectory, autoStep + 1);
    }

    /**
     * @param seconds How many seconds in must've passed to call fn
     * @param fn      An unnamed function that'll be called if `seconds` has passed.
     */
    private static void doAfterTime(double seconds, UnamedFunction fn) {
        if (miniTimer.get() == 0.0) {
            miniTimer.start();
        }

        if (miniTimer.hasElapsed(seconds)) {
            fn.callback();
        }
    }

    /**
     * Note that this function resets the timer after `seconds` has elapsed.
     * 
     * @param seconds      How many seconds to wait before going to `nextAutoStep`
     * @param nextAutoStep The next autoStep
     * @param fn           An unnamed function that'll be called if `seconds`
     *                     hasn't passed.
     */
    private static void doOnTimer(double seconds, int nextAutoStep, UnamedFunction fn) {
        if (!autoTimer.hasElapsed(seconds)) {
            fn.callback();
        } else {
            incAutoStep();
        }
    }

    private static void doOnTimer(double seconds, UnamedFunction fn) {
        doOnTimer(seconds, autoStep + 1, fn);
    }

    // Idea psuedo-code
    // public static void genAutoRoutine(
    // // {
    // // traj,
    // // {5, () -> {
    // // // stuff
    // // }}
    // // traj2,
    // }

    // This is for the "Default" auto state as we want it to do nothing
    public static void nothing() {
    }

    public static void centerBalance() {
        final PathPlannerTrajectory first = AutoStates.CENTER_BALANCE.paths.get(0);

        switch (autoStep) {
            case 0:
                // Reset the drivetrain's odometry to the starting pose of the trajectory.
                setNavigationToTrajectoryStart(first);
                incAutoStep();
                break;
            case 1:
                doTrajectory(first);
                break;
            case 2:
                RobotContainer.driveTrain.setBrakeMode();
                incAutoStep();
                break;
            case 3:
                RobotContainer.driveTrain.balanceChargeStation();
                break;
        }
    }

    public static void centerBalanceKick() {
        final PathPlannerTrajectory first = AutoStates.CENTER_BALANCE_KICK.paths.get(0);

        switch (autoStep) {
            case 0:
                // Reset the drivetrain's odometry to the starting pose of the trajectory.
                setNavigationToTrajectoryStart(first);
                incAutoStep();
                break;
            case 1:
                doOnTimer(0.5, () -> {
                    RobotContainer.arm.setKickerOn();
                });
            case 2:
                RobotContainer.arm.setKickerOff();
                incAutoStep();
            case 3:
                doTrajectory(first);
                break;
            case 4:
                RobotContainer.driveTrain.setBrakeMode();
                incAutoStep();
                break;
            case 5:
                RobotContainer.driveTrain.balanceChargeStation();
                break;
        }
    }

    public static void rightScoreRed() {
        final PathPlannerTrajectory first = AutoStates.RIGHT_SCORE_RED.paths.get(0);
        final PathPlannerTrajectory second = AutoStates.RIGHT_SCORE_RED.paths.get(1);

        switch (autoStep) {
            case 0:
                // Reset the drivetrain's odometry to the starting pose of the trajectory.
                setNavigationToTrajectoryStart(first);
                incAutoStep();
                break;
            case 1:
                doOnTimer(0.5, () -> {
                    RobotContainer.arm.setKickerOn();
                });
                break;
            case 2:
                RobotContainer.arm.gotoTravel();
                doTrajectory(first);
                doAfterTime(0.5, () -> {
                    RobotContainer.intake.setIntakeSolenoid(true);
                });
                doAfterTime(0.9, () -> {
                    RobotContainer.intake.runIntake(SubIntakeModes.CUBE);
                    RobotContainer.arm.openClaw();
                });
                break;
            case 3:
                doTrajectory(second);
                RobotContainer.intake.stopIntake();
                RobotContainer.arm.gotoTravel();
                break;
            case 4:
                RobotContainer.arm.resetMotionMagicTimer();
                incAutoStep();
                break;
            case 5:
                doOnTimer(3, () -> {
                    RobotContainer.arm.gotoPlatter();
                });
                break;
            case 6:
                doOnTimer(0.5, () -> {
                    RobotContainer.arm.closeClaw();
                    RobotContainer.arm.gotoPlatter();
                });
                break;
            case 7:
                RobotContainer.arm.resetMotionMagicTimer();
                incAutoStep();
                break;
            case 8:
                doOnTimer(3, () -> {
                    RobotContainer.arm.gotoMid();
                });
                break;
            case 9:
                RobotContainer.arm.gotoMid();
                RobotContainer.arm.openClaw();
                break;
        }

    }

    // TODO: FLIP PATH
    public static void leftScoreRed() {
        // final PathPlannerTrajectory first = AutoStates.LEFT_SCORE_RED.paths.get(0);
        // final PathPlannerTrajectory second = AutoStates.LEFT_SCORE_RED.paths.get(1);

        // switch (autoStep) {
        // case 0:
        // // Reset the drivetrain's odometry to the starting pose of the trajectory.
        // setNavigationToTrajectoryStart(first);
        // resetAutoTimer();
        // autoStep++;
        // break;
        // case 1:
        // doOnTimer(0.5, autoStep + 1, () -> {
        // RobotContainer.arm.setKickerOn();
        // });
        // break;
        // case 2:
        // RobotContainer.arm.gotoTravel();
        // doTrajectory(first, autoStep + 1);
        // doAfterTime(0.5, () -> {
        // RobotContainer.intake.setIntakeSolenoid(true);
        // });
        // doAfterTime(0.9, () -> {
        // RobotContainer.intake.runIntake(SubIntakeModes.CUBE);
        // RobotContainer.arm.openClaw();
        // });
        // break;
        // case 3:
        // doTrajectory(second, autoStep + 1);
        // RobotContainer.intake.stopIntake();
        // RobotContainer.arm.gotoTravel();
        // break;
        // case 4:
        // RobotContainer.arm.resetMotionMagicTimer();
        // autoStep++;
        // break;
        // case 5:
        // doOnTimer(3, autoStep + 1, () -> {
        // RobotContainer.arm.gotoPlatter();
        // });
        // break;
        // case 6:
        // doOnTimer(0.5, autoStep + 1, () -> {
        // RobotContainer.arm.closeClaw();
        // RobotContainer.arm.gotoPlatter();
        // });
        // break;
        // case 7:
        // RobotContainer.arm.resetMotionMagicTimer();
        // autoStep++;
        // break;
        // case 8:
        // doOnTimer(3, autoStep + 1, () -> {
        // RobotContainer.arm.gotoMid();
        // });
        // break;
        // case 9:
        // RobotContainer.arm.gotoMid();
        // RobotContainer.arm.openClaw();
        // break;
        // }

    }

    public static void leftScore() {
        // final PathPlannerTrajectory first = AutoStates.LEFT_SCORE.paths.get(0);
        // final PathPlannerTrajectory second = AutoStates.LEFT_SCORE.paths.get(1);

        // switch (autoStep) {
        // case 0:
        // // Reset the drivetrain's odometry to the starting pose of the trajectory.
        // setNavigationToTrajectoryStart(first);
        // resetAutoTimer();
        // autoStep++;
        // break;
        // case 1:
        // doOnTimer(5, autoStep + 1, () -> {
        // // score the cone
        // });
        // break;
        // case 2:
        // doTrajectory(first, autoStep + 1);
        // break;
        // case 3:
        // doTrajectory(second, autoStep + 1);
        // break;
        // case 4:
        // doOnTimer(5, -1, () -> {
        // // score cube
        // });
        // break;
        // }
    }

    public static void rightBalance() {
        final PathPlannerTrajectory first = AutoStates.RIGHT_BALANCE.paths.get(0);
        final PathPlannerTrajectory second = AutoStates.RIGHT_BALANCE.paths.get(1);

        switch (autoStep) {
            case 0:
                // Reset the drivetrain's odometry to the starting pose of the trajectory.
                setNavigationToTrajectoryStart(first);
                resetAutoTimer();
                incAutoStep();
                break;
            case 1:
                doOnTimer(5, () -> {
                    // score the cone
                });
                break;
            case 2:
                doTrajectory(first);
                break;
            case 3:
                doTrajectory(second);
                break;
            case 4:
                RobotContainer.driveTrain.setBrakeMode();
                incAutoStep();
                break;
            case 5:
                RobotContainer.driveTrain.balanceChargeStation();
                break;
        }
    }

    public static void leftBalance() {
        final PathPlannerTrajectory first = AutoStates.LEFT_BALANCE.paths.get(0);
        final PathPlannerTrajectory second = AutoStates.LEFT_BALANCE.paths.get(1);

        switch (autoStep) {
            case 0:
                // Reset the drivetrain's odometry to the starting pose of the trajectory.
                setNavigationToTrajectoryStart(first);
                incAutoStep();
                break;
            case 1:
                doOnTimer(5, () -> {
                    // score the cone
                });
                break;
            case 2:
                doTrajectory(first);
                break;
            case 3:
                doOnTimer(5, () -> {
                    // pick up the cube
                });
                break;
            case 4:
                doTrajectory(second);
                break;
            case 5:
                RobotContainer.driveTrain.setBrakeMode();
                incAutoStep();
                break;
            case 6:
                RobotContainer.driveTrain.balanceChargeStation();
                break;
        }
    }
}
