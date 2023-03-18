package frc.robot.subsystems;

import frc.robot.Constants;

public enum SubIntakeModes {
    CONE("cone", false, Constants.Intake_Settings.CONE_INTAKE_SPEED),
    CUBE("cube", false, Constants.Intake_Settings.CUBE_INTAKE_SPEED, 0),
    CUBE_LIMITED("cubeLimited", true, Constants.Intake_Settings.CUBE_INTAKE_SPEED, 0),
    REVERSE("reverse", false, Constants.Intake_Settings.REVERSE),
    EJECT_MID("ejectMid", false, Constants.Intake_Settings.MID_EJECT),
    EJECT_HIGH("ejectHigh", false, Constants.Intake_Settings.HIGH_EJECT);

    private final String label;
    public final double topSpeed;
    public final double bottomSpeed;
    public final boolean limited;

    private SubIntakeModes(String label, boolean limited, double topSpeed, double bottomSpeed) {
        this.label = label;
        this.limited = limited;
        this.topSpeed = topSpeed;
        this.bottomSpeed = bottomSpeed;
    }

    private SubIntakeModes(String label, boolean limited, double speed) {
        this(label, limited, speed, speed);
    }

    @Override
    public String toString() {
        return this.label;
    }
}
