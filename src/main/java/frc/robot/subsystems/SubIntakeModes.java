package frc.robot.subsystems;

public enum SubIntakeModes {
    CUBE_LIMITED("limited_cube"),
    CUBE("cube"),
    CONE("cone"),
    REVERSE("reverse"),
    EJECT_MID("eject_mid"),
    EJECT_HIGH("eject_high");

    private String label;

    private SubIntakeModes(String label) {
        this.label = label;
    }

    @Override
    public String toString() {
        return this.label;
    }
}
