package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class CustomController extends XboxController {
    public CustomController(int port) {
        super(port);
    }

    @Override
    public double getLeftY() {
        return Math.pow(joyDeadband(super.getLeftY()), 2) * Math.signum(super.getLeftY());
    }

    @Override
    public double getRightY() {
        return Math.pow(joyDeadband(super.getRightY()), 2) * Math.signum(super.getRightY());
    }

    @Override
    public double getRightX() {
        return Math.pow(joyDeadband(super.getRightX()), 2) * Math.signum(super.getRightX());
    }

    public boolean getRightTrigger() {
        return super.getRightTriggerAxis() > 0.1;
    }

    public boolean getLeftTrigger() {
        return super.getLeftTriggerAxis() > 0.1;
    }

    public boolean getPOVUp() {
        return super.getPOV() < 0 + Constants.Misc.CONTROLLER_POV_MARGIN
                && super.getPOV() > 0 - Constants.Misc.CONTROLLER_POV_MARGIN;
    }

    public boolean getPOVRight() {
        return super.getPOV() < 90 + Constants.Misc.CONTROLLER_POV_MARGIN
                && super.getPOV() > 90 - Constants.Misc.CONTROLLER_POV_MARGIN;
    }

    public boolean getPOVDown() {
        return super.getPOV() < 180 + Constants.Misc.CONTROLLER_POV_MARGIN
                && super.getPOV() > 180 - Constants.Misc.CONTROLLER_POV_MARGIN;
    }

    public boolean getPOVLeft() {
        return super.getPOV() < 270 + Constants.Misc.CONTROLLER_POV_MARGIN
                && super.getPOV() > 270 - Constants.Misc.CONTROLLER_POV_MARGIN;
    }

    /*
     * Joystick deadband method makes sure the joysticks are set to zero when
     * released
     */
    private double joyDeadband(double stickValue) {
        if (Math.abs(stickValue) <= Constants.Misc.CONTROLLER_DEADBAND) {
            return 0;
        } else {
            return stickValue;
        }
    }
}
