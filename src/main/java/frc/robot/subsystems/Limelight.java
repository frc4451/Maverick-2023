package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class Limelight {
    private final NetworkTable table;

    private final PIDController TURN_CONTROLLER = new PIDController(
            Constants.DT_Settings.TARGETING_TURNING_PG,
            Constants.DT_Settings.TARGETING_TURNING_IG,
            Constants.DT_Settings.TARGETING_TURNING_DG);

    public Limelight(String tableKey) {
        table = NetworkTableInstance.getDefault().getTable(tableKey);

        // We always want these settings for TURN_CONTROLLER
        this.TURN_CONTROLLER.setSetpoint(0);
        this.TURN_CONTROLLER.setTolerance(1);
    }

    // Epic boilerplate per:
    // https://docs.limelightvision.io/en/latest/getting_started.html#basic-programming
    /**
     * @return Whether the limelight has any valid targets
     */
    public boolean hasTargets() {
        return this.table.getEntry("tv").getBoolean(false);
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
    //     return this.table.getEntry("ty").getDouble(0.0);
    // }

    // We can make these constants later if needed.
    public void setAimingPipelineEnabled(boolean isEnabled) {
        if (isEnabled) {
            this.table.getEntry("pipeline").setNumber(1);
        } else {
            this.table.getEntry("pipeline").setNumber(0);
        }
    }

    public double getTurnFromLimelight() {
        if (this.hasTargets()) {
            return this.TURN_CONTROLLER.calculate(this.getXOffset()) * Constants.DT_Settings.MAX_VELOCITY;
        } else {
            return 0;
        }
    }
}
