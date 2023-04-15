/*
 * AutoContainer holds public static methods for autonomous routines
 * Each static method will be one entire autonomous routine called in Robot.autonomousPeriodic()
 * The autonomous mode is selected from the dashboard through a SendableChooser
 */
package frc.robot.auto;

import java.util.List;

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
        if (autoTimer.get() <= trajectory.getTotalTimeSeconds()) {
            // Get the desired pose from the trajectory.
            PathPlannerState desiredPose = (PathPlannerState) trajectory.sample(autoTimer.get());

            // Get the reference chassis speeds from the Ramsete controller.
            ChassisSpeeds refChassisSpeeds = RobotContainer.driveTrain.ramseteCalculate(desiredPose);

            // Set the linear and angular speeds.
            RobotContainer.driveTrain.drive(refChassisSpeeds);
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

    private static void doUntilTime(double seconds, UnamedFunction fn) {
        if (miniTimer.get() == 0.0) {
            miniTimer.start();
        }

        if (!miniTimer.hasElapsed(seconds)) {
            fn.callback();
        }
    }

    private static void doInbetweenTime(double start, double end, UnamedFunction fn) {
        final double currentTime = miniTimer.get();

        if (currentTime == 0.0) {
            miniTimer.start();
        }

        if (currentTime >= start && currentTime <= end) {
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

    public static void lonelyKick() {
        RobotContainer.arm.setKickerOn();
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
                break;
            case 2:
                doTrajectory(first);
                break;
            case 3:
                RobotContainer.driveTrain.setBrakeMode();
                incAutoStep();
                break;
            case 4:
                RobotContainer.driveTrain.balanceChargeStation();
                break;
        }
    }

    public static void centerBalancePieceBlue() {
        centerBalanceAndPiece(AutoStates.CENTER_BALANCE_PIECE_BLUE.paths);
    }

    public static void centerBalancePieceRed() {
        centerBalanceAndPiece(AutoStates.CENTER_BALANCE_PIECE_RED.paths);
    }

    public static void centerBalanceAndPiece(List<PathPlannerTrajectory> paths) {
        final PathPlannerTrajectory first = paths.get(0);

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
                doTrajectory(first);
                RobotContainer.arm.gotoTravel();

                doInbetweenTime(3.5, 3.9, () -> {
                    RobotContainer.intake.setIntakeSolenoid(true);
                });

                doInbetweenTime(3.9, 5.9, () -> {
                    RobotContainer.intake.runIntake(SubIntakeModes.CUBE_LIMITED);
                });

                doAfterTime(6.0, () -> {
                    RobotContainer.intake.stopIntake();
                    RobotContainer.intake.setIntakeSolenoid(false);
                });
                break;
            case 3:
                RobotContainer.driveTrain.balanceChargeStation();
                break;
        }
    }

    public static void leftScoreBlue() {
        kickOutGrabCubeBack(AutoStates.LEFT_SCORE_BLUE.paths);
    }

    public static void leftScoreRed() {
        kickOutGrabCubeBack(AutoStates.LEFT_SCORE_RED.paths);
    }

    // public static void rightScoreBlue() {
    // kickOutGrabCubeBack(AutoStates.RIGHT_SCORE_BLUE.paths);
    // }

    public static void rightScoreRed() {
        kickOutGrabCubeBack(AutoStates.RIGHT_SCORE_RED.paths);
    }

    public static void kickOutGrabCubeBack(List<PathPlannerTrajectory> paths) {
        PathPlannerTrajectory first = paths.get(0);
        PathPlannerTrajectory second = paths.get(1);

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
                RobotContainer.intake.runIntake(SubIntakeModes.CUBE);
                RobotContainer.arm.gotoTravel();
                break;
            case 4:
                RobotContainer.arm.resetMotionMagicTimer();
                RobotContainer.intake.stopIntake();
                incAutoStep();
                break;
            case 5:
                doOnTimer(1.5, () -> {
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
}
