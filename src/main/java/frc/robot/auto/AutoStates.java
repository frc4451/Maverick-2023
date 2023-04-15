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
    NOTHING(
            "Nothing",
            AutoContainer::nothing,
            PathPlanner.loadPathGroup("nothing", Speeds.none)),
    LONELY_KICK(
            "Lonely Kick",
            AutoContainer::lonelyKick,
            PathPlanner.loadPathGroup("nothing", Speeds.none)),
    LEFT_SCORE_BLUE(
            "Left Score Blue",
            AutoContainer::leftScoreBlue,
            PathPlanner.loadPathGroup("leftScoreBlue", Speeds.fast)),
    LEFT_SCORE_RED(
            "Left Score Red",
            AutoContainer::leftScoreRed,
            PathPlanner.loadPathGroup("leftScoreRed", Speeds.fast)),
    CENTER_BALANCE(
            "Center Balance",
            AutoContainer::centerBalance,
            PathPlanner.loadPathGroup("centerBalance", Speeds.centerBalance)),
    CENTER_BALANCE_KICK(
            "Center Balance Kick",
            AutoContainer::centerBalanceKick,
            PathPlanner.loadPathGroup("centerBalanceKick", Speeds.centerBalance)),
    CENTER_BALANCE_PIECE_BLUE(
            "Center Balance Piece Blue",
            AutoContainer::centerBalancePieceBlue,
            PathPlanner.loadPathGroup("centerBalancePieceBlue", Speeds.centerBalance)),
    CENTER_BALANCE_PIECE_RED(
            "Center Balance Piece Red",
            AutoContainer::centerBalancePieceRed,
            PathPlanner.loadPathGroup("centerBalancePieceRed", Speeds.centerBalance)),
    // RIGHT_SCORE_BLUE(
    // "Right Score Blue",
    // AutoContainer::rightScoreBlue,
    // PathPlanner.loadPathGroup("rightScoreBlue", Speeds.medium)),
    RIGHT_SCORE_RED("Right Score Red",
            AutoContainer::rightScoreRed,
            PathPlanner.loadPathGroup("rightScoreRed", Speeds.medium));

    public final String label;

    public final UnamedFunction routine;

    public final List<PathPlannerTrajectory> paths;

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