// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.robot.Constants;

/** Add your docs here. */
public class SubDriveTrain {

    private final WPI_TalonFX LEFT_FRONT;
    private final WPI_TalonFX LEFT_BACK;
    private final WPI_TalonFX RIGHT_FRONT;
    private final WPI_TalonFX RIGHT_BACK;
    
    // private final GYRO GYROSCOPE;

    // private final WPI_GYRO GYROSCOPE;
    private final DifferentialDriveKinematics KINEMATICS;
    private final DifferentialDrivePoseEstimator ODOMETRY;

    private final RamseteController RAMSETE = new RamseteController(Constants.Autonomous.BETA,
            Constants.Autonomous.ZETA);
    // Speed Controllers
    private final PIDController LEFT_PID_CONTROLLER = new PIDController(
            Constants.DT_PIDF.LEFT_PG,
            Constants.DT_PIDF.LEFT_IG,
            Constants.DT_PIDF.LEFT_DG);

    private final PIDController RIGHT_PID_CONTROLLER = new PIDController(
            Constants.DT_PIDF.RIGHT_PG,
            Constants.DT_PIDF.RIGHT_IG,
            Constants.DT_PIDF.RIGHT_DG);

    private final SimpleMotorFeedforward LEFT_FEED_FORWARD = new SimpleMotorFeedforward(
            Constants.DT_PIDF.STATIC_GAIN,
            Constants.DT_PIDF.VELOCITY_GAIN,
            Constants.DT_PIDF.ACCEL_GAIN);

    private final SimpleMotorFeedforward RIGHT_FEED_FORWARD = new SimpleMotorFeedforward(
            Constants.DT_PIDF.STATIC_GAIN,
            Constants.DT_PIDF.VELOCITY_GAIN,
            Constants.DT_PIDF.ACCEL_GAIN);
    
    public SubDriveTrain(int leftFront, int leftBack, int rightFront, int rightBack, int gyro) {
        // Instance devices
        this.LEFT_FRONT = new WPI_TalonFX(leftFront);
        this.LEFT_BACK = new WPI_TalonFX(leftBack);
        this.RIGHT_FRONT = new WPI_TalonFX(rightFront);
        this.RIGHT_BACK = new WPI_TalonFX(rightBack);
        this.KINEMATICS = new DifferentialDriveKinematics(
            TechnicalConstants.DT_WIDTH);
        this.ODOMETRY = new DifferentialDrivePoseEstimator(
            this.KINEMATICS,
            Rotation2d.fromDegrees(0),
            this.getLeftEncoderMeters(),
            this.getRightEncoderMeters(),
            new Pose2D());
        
        // Reset devices to factory default
        this.LEFT_FRONT.configFactoryDefault();
        this.LEFT_BACK.configFactoryDefault();
        this.RIGHT_FRONT.configFactoryDefault();
        this.RIGHT_BACK.configFactoryDefault();
        this.GYROSCOPE.reset();

        // Follower motors
        this.LEFT_BACK.follow(this.LEFT_FRONT);
        this.RIGHT_BACK.follow(this.RIGHT_FRONT);

        this.LEFT_FRONT.setSensorPhase(false);
        this.RIGHT_FRONT.setSensorPhase(false);

        // Invert right side
        this.LEFT_FRONT.setInverted(false);
        this.LEFT_BACK.setInverted(false);
        this.RIGHT_FRONT.setInverted(true);
        this.RIGHT_BACK.setInverted(true);

        // Set motors to coast mode
        this.LEFT_FRONT.setNeutralMode(NeutralMode.Coast);
        this.LEFT_BACK.setNeutralMode(NeutralMode.Coast);
        this.RIGHT_FRONT.setNeutralMode(NeutralMode.Coast);
        this.RIGHT_BACK.setNeutralMode(NeutralMode.Coast);

        // Time from neutral to full throttle
        this.LEFT_FRONT.configClosedLoopRamp(DT_Settings.RAMP_RATE_SECS);
        this.RIGHT_FRONT.configClosedLoopRamp(DT_Settings.RAMP_RATE_SECS);
        this.LEFT_FRONT.configPeakOutputForward(1, DT_PIDF.TIMEOUT_MS);

        // Peak and nominal outputs
        this.LEFT_FRONT.configNominalOutputForward(0, DT_PIDF.TIMEOUT_MS);
        this.LEFT_FRONT.configNominalOutputReverse(0, DT_PIDF.TIMEOUT_MS);
        this.LEFT_FRONT.configPeakOutputForward(1, DT_PIDF.TIMEOUT_MS);
        this.LEFT_FRONT.configPeakOutputReverse(-1, DT_PIDF.TIMEOUT_MS);

        this.RIGHT_FRONT.configNominalOutputForward(0, DT_PIDF.TIMEOUT_MS);
        this.RIGHT_FRONT.configNominalOutputReverse(0, DT_PIDF.TIMEOUT_MS);
        this.RIGHT_FRONT.configPeakOutputForward(1, DT_PIDF.TIMEOUT_MS);
        this.RIGHT_FRONT.configPeakOutputReverse(-1, DT_PIDF.TIMEOUT_MS);

        // Motion Magic
        this.LEFT_FRONT.configMotionAcceleration(DT_Settings.MM_ACCELERATION);
        this.LEFT_FRONT.configMotionCruiseVelocity(DT_Settings.MM_CRUISECONTROL);
        this.RIGHT_FRONT.configMotionAcceleration(DT_Settings.MM_ACCELERATION);
        this.RIGHT_FRONT.configMotionCruiseVelocity(DT_Settings.MM_CRUISECONTROL);
        // PID
        this.LEFT_FRONT.config_kP(
                DT_PIDF.PID_LOOP_INDEX,
                DT_PIDF.LEFT_PG,
                DT_PIDF.TIMEOUT_MS);
        this.LEFT_FRONT.config_kI(
                DT_PIDF.PID_LOOP_INDEX,
                DT_PIDF.LEFT_IG,
                DT_PIDF.TIMEOUT_MS);
        this.LEFT_FRONT.config_kD(
                DT_PIDF.PID_LOOP_INDEX,
                DT_PIDF.LEFT_DG,
                DT_PIDF.TIMEOUT_MS);
        this.LEFT_FRONT.config_kF(
                DT_PIDF.PID_LOOP_INDEX,
                DT_PIDF.LEFT_FF,
                DT_PIDF.TIMEOUT_MS);

        this.RIGHT_FRONT.config_kP(
                DT_PIDF.PID_LOOP_INDEX,
                DT_PIDF.RIGHT_PG,
                DT_PIDF.TIMEOUT_MS);
        this.RIGHT_FRONT.config_kI(
                DT_PIDF.PID_LOOP_INDEX,
                DT_PIDF.RIGHT_IG,
                DT_PIDF.TIMEOUT_MS);
        this.RIGHT_FRONT.config_kD(
                DT_PIDF.PID_LOOP_INDEX,
                DT_PIDF.RIGHT_DG,
                DT_PIDF.TIMEOUT_MS);
        this.RIGHT_FRONT.config_kF(
                DT_PIDF.PID_LOOP_INDEX,
                DT_PIDF.RIGHT_FF,
                DT_PIDF.TIMEOUT_MS);
    }

    // DRIVE METHODS
    public void runDrive(double forward, double rotate, boolean quickTurnEnabled) {

    }
    public void runMotionMagic(double targetDistanceL, double targetDistanceR) {
        this.LEFT_FRONT.set(ControlMode.MotionMagic, targetDistanceL);
        this.RIGHT_FRONT.set(ControlMode.MotionMagic, targetDistanceR);
    }
    public void kinematicDrive(double linearVelocity, double angularVelocity) {
        DifferentialDriveWheelSpeeds wheelSpeeds = KINEMATICS.toWheelSpeeds(new ChassisSpeeds(linearVelocity, 0.0, angularVelocity));
        setWheelSpeeds(wheelSpeeds);
    }

    /**
     * getter methods to display values on dashboard
     */
    public double getLeftSpeed() {
        return this.LEFT_FRONT.getSelectedSensorVelocity();
    }
    public double getLeftPosition() {
        return this.LEFT_FRONT.getSelectedSensorPosition();
    }

    public double getRightSpeed() {
        return this.RIGHT_FRONT.getSelectedSensorVelocity();
    }
    public double getRightDistance() {
        return this.RIGHT_FRONT.getSelectedSEnsorPosition();
    }

    public double getGyroAngle() {
        return (this.GYROSCOPE.getAngle() % 360); // Gyroscope relative rotation
    }
    // KINEMATICS/ODOMETRY/ETC..

    private void setWheelSpeeds() {

    }

}
