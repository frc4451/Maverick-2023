package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.util.RobotMath;

public class Limelight {
    private final NetworkTable table;

    private int currentPipeline; // TODO: make set methods that update this whenever they're called, instead of
                                 // setting the table directly.

    // private final PIDController TURN_CONTROLLER = new PIDController(
    // Constants.DT_Settings.TARGETING_TURNING_PG,
    // Constants.DT_Settings.TARGETING_TURNING_IG,
    // Constants.DT_Settings.TARGETING_TURNING_DG);

    public Limelight(String tableKey) {
        table = NetworkTableInstance.getDefault().getTable(tableKey);
        currentPipeline = 0;

        // We always want these settings for TURN_CONTROLLER
        // this.TURN_CONTROLLER.setSetpoint(0);
        // this.TURN_CONTROLLER.setTolerance(2);
    }

    // public void resetTurnController() {
    // this.TURN_CONTROLLER.reset();
    // }

    // Epic boilerplate per:
    // https://docs.limelightvision.io/en/latest/getting_started.html#basic-programming
    /**
     * @return Whether the limelight has any valid targets
     */
    public boolean hasTargets() {
        return this.table.getEntry("tv").getInteger(0) == 1;
    }

    /**
     * @return Horizontal Offset From Crosshair To Target (-27 to 27 degrees)
     */
    public double getXOffset() {
        return this.table.getEntry("tx").getDouble(0.0);
    }

    /**
     * @return Vertical Offset From Crosshair To Target (-20.5 to 20.5 degrees)
     */
    // public double getYOffset() {
    // return this.table.getEntry("ty").getDouble(0.0);
    // }

    // We can make these constants later if needed.
    public void setAimingPipelineEnabled(boolean isEnabled) {
        // if (isEnabled) {
        // this.table.getEntry("pipeline").setNumber(1);
        // } else {
        // this.table.getEntry("pipeline").setNumber(0);
        // }

        // 0 is regular camera, 1 is targeting with half of limelight lights
        // Mhhhhhhhh inline if else
        this.table.getEntry("pipeline").setNumber(isEnabled ? 1 : 0);
    }

    public void toggleAimingPipeline() {
        currentPipeline = (currentPipeline == 1 ? 0 : 1);
        this.table.getEntry("pipeline").setNumber(currentPipeline);
    }

    // public int getCurrentPipeline() {
    // return this.table.getEntry("pipeline").getInteger(0);
    // }

    public double getTurnFromLimelight() {
        double error = -this.getXOffset();
        boolean ifCondition = this.hasTargets()
                && this.table.getEntry("pipeline").getInteger(0) != 0
                && (Math.abs(error) >= 2.0);
        return ifCondition ? (-Math.signum(error) * Constants.DT_Settings.TARGETING_TURNING_SG) : 0;
    }

    // public double getTurnFromLimelight() {
    // double error = -this.getXOffset();
    // if (this.hasTargets()) {
    // return RobotMath.clamp(
    // this.TURN_CONTROLLER.calculate(error)
    // - (Math.signum(error) * Constants.DT_Settings.TARGETING_TURNING_SG),
    // -0.4, 0.4);
    // } else {
    // return 0;
    // }
    // }
}
