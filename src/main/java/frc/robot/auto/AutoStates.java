package frc.robot.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

/**
 * Class that's only purpose is to store PathConstraints objects because they
 * couldn't be defined in the AutoStates enum due to forward references.
 * 
 * @see AutoStates
 */
class Speeds {
    public static final PathConstraints none = new PathConstraints(0.0, 0.0);
    public static final PathConstraints fast = new PathConstraints(2.5, 2.0);
    public static final PathConstraints medium = new PathConstraints(2.0, 2.0);
    public static final PathConstraints slow = new PathConstraints(1.5, 2.0);
    public static final PathConstraints centerBalance = new PathConstraints(1.2, 1.2);
}

public enum AutoStates {
    DEFAULT(
            "Default",
            AutoContainer::nothing,
            PathPlanner.loadPathGroup("nothing", Speeds.none)),
    LEFT_BALANCE(
            "Left Balance",
            AutoContainer::leftBalance,
            PathPlanner.loadPathGroup("leftBalance", Speeds.slow)),
    LEFT_SCORE_BLUE(
            "Left Score Blue",
            AutoContainer::leftScoreBlue,
            PathPlanner.loadPathGroup("leftScoreBlue", Speeds.fast)),
    CENTER_BALANCE(
            "Center Balance",
            AutoContainer::centerBalance,
            PathPlanner.loadPathGroup("centerBalance", Speeds.centerBalance)),
    CENTER_BALANCE_KICK(
            "Center Balance Kick",
            AutoContainer::centerBalanceKick,
            PathPlanner.loadPathGroup("centerBalanceKick", Speeds.centerBalance)),
    RIGHT_BALANCE(
            "Right Balance",
            AutoContainer::rightBalance,
            PathPlanner.loadPathGroup("rightBalance", Speeds.fast)),
    RIGHT_SCORE_RED(
            "Right Score Red",
            AutoContainer::rightScoreRed,
            PathPlanner.loadPathGroup("rightScoreRed", Speeds.medium));
    // BOTTOM_SCORE_RED( // This shouldn't be needed
    // "Bottom Score Red",
    // AutoContainer::bottomScore,
    // PathPlanner.loadPathGroup("bottomScoreRed", Speeds.medium));

    public final String label;

    public final UnamedFunction routine;

    public final List<PathPlannerTrajectory> paths;

    /**
     * Static int storing the longest path group in the enum.
     * This variable's intended purpose is for Shuffleboard
     * to know how many Trajectory's it needs to deal with.
     */
    public final static int longestPathGroup = getLongestGroup();

    /**
     * We use this for the ShuffleBoard field widget to properly display paths
     * 
     * @see #longestPathGroup
     */
    private static int getLongestGroup() {
        int longest = 0;
        for (AutoStates state : AutoStates.values()) {
            if (state.paths.size() > longest) {
                longest = state.paths.size();
            }
        }
        return longest;
    }

    /**
     * @param label   Label that is mainly used for Shuffleboard
     * @param routine Function (or reference) that will be called periodically by
     *                this AutoState
     * @param paths   List of PathPlannerTrajectory's that is usually generated with
     *                {@code PathPlanner.loadPathGroup()}
     */
    private AutoStates(String label, UnamedFunction routine, List<PathPlannerTrajectory> paths) {
        this.label = label;
        this.routine = routine;
        this.paths = paths;
        // this.paths = paths.stream()
        // .map(path -> PathPlannerTrajectory.transformTrajectoryForAlliance(
        // path,
        // DriverStation.getAlliance()))
        // .collect(Collectors.toList());
    }

    @Override
    public String toString() {
        return this.label;
    }
}