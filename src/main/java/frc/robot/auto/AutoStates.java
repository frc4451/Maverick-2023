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
    public static final PathConstraints slow = new PathConstraints(1.5, 1.0);
    public static final PathConstraints verySlow = new PathConstraints(1.0, 1.0);
}

public enum AutoStates {
    DEFAULT(
            "Default",
            AutoContainer::nothing,
            PathPlanner.loadPathGroup("nothing", Speeds.none)),
    LEFT_BALANCE(
            "Left Balance",
            AutoContainer::leftBalance,
            PathPlanner.loadPathGroup("leftBalance", Speeds.fast)),
    LEFT_SCORE(
            "Left Score",
            AutoContainer::leftScore,
            PathPlanner.loadPathGroup("leftScore", Speeds.fast)),
    CENTER_BALANCE(
            "Center Balance",
            AutoContainer::centerBalance,
            PathPlanner.loadPathGroup("centerBalance", Speeds.verySlow)),
    RIGHT_BALANCE(
            "Right Balance",
            AutoContainer::rightBalance,
            PathPlanner.loadPathGroup("rightBalance", Speeds.fast)),
    RIGHT_SCORE(
            "Right Score",
            AutoContainer::rightScore,
            PathPlanner.loadPathGroup("rightScore", Speeds.medium));

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
    }

    @Override
    public String toString() {
        return this.label;
    }
}