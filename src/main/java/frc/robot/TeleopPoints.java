package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Enum containing predefined points on the field that can be driven to.
 */
enum TeleopPoints {
    // These are arbitrary numbers for now
    TEST_POINT(
            new Pose2d(2.0, 1.0, new Rotation2d(0)), // Blue
            new Pose2d(3.0, 4.0, new Rotation2d(0))); // Red

    TeleopPoints(Pose2d blue, Pose2d red) {
        this.blue = blue;
        this.red = red;
    }

    private final Pose2d red;
    private final Pose2d blue;

    /**
     * @return Blue point
     */
    public Pose2d getBlue() {
        return this.blue;
    }

    /**
     * @return Red point
     */
    public Pose2d getRed() {
        return this.red;
    }

    /**
     * @return Either Blue or Red point depending on the alliance reported by DriverStation.
     */
    public Pose2d getForAlliance() {
        return DriverStation.getAlliance() == Alliance.Blue ? this.getBlue() : this.getRed();
    }

    /**
     * Drives to the point using the {@link RamseteController} in {@link RobotContainer}'s Drivetrain
     */
    public void driveTowards() {
        Pose2d point = this.getForAlliance();
        ChassisSpeeds refChassisSpeeds = RobotContainer.driveTrain.ramseteCalculate(point, 1.0, Math.PI / 2.0);
        RobotContainer.driveTrain.drive(refChassisSpeeds);
    }
}
