package frc.robot.subsystems;

public enum SubIntakeModes {
    CUBE_LIMITED("limited_cube"),
    CUBE("cube"),
    CONE("cone"),
    REVERSE("reverse");

    private String label;

    private SubIntakeModes(String label) {
        this.label = label;
    }

    @Override
    public String toString() {
        return this.label;
    }
}
