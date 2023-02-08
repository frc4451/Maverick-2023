/*
SubDriveTrain class defines the drive train object and methods used to control and monitor status
    Skid steer tank drive with two Falcon500 motors per side
    A Pigeon IMU is used as the primary gyro for monitoring and controlling robot heading
 */
package frc.robot.subsystems;

// Import Constants and CRTE libraries for the Falcon motors and Pigeon IMU
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.apriltag.AprilTagDetector.Config;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.util.RobotMath;
import frc.robot.Constants.DT_PIDF;
import frc.robot.Constants.DT_Settings;
import frc.robot.Constants.TechnicalConstants;

public class SubDriveTrain {
    // Class variable definitions. Define the variable names for the WPI_TalonFX.
    private final WPI_TalonFX LEFT_FRONT;
    private final WPI_TalonFX RIGHT_FRONT;
    private final WPI_TalonFX LEFT_REAR;
    private final WPI_TalonFX RIGHT_REAR;
    private final WPI_PigeonIMU GYROSCOPE;
    private final WPI_TalonFX MOTOR5;

    private final DifferentialDriveKinematics KINEMATICS;
    private final DifferentialDrivePoseEstimator ODOMETRY;

    private final RamseteController RAMSETE = new RamseteController(Constants.Auto.BETA, Constants.Auto.ZETA);

    // speed controllers
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
            Constants.DT_PIDF.ACCELERATION_GAIN);

    private final SimpleMotorFeedforward RIGHT_FEED_FORWARD = new SimpleMotorFeedforward(
            Constants.DT_PIDF.STATIC_GAIN,
            Constants.DT_PIDF.VELOCITY_GAIN,
            Constants.DT_PIDF.ACCELERATION_GAIN);

    /*
     * Drive train constructor
     * Receives CAN address of 4 drive train Falcon motors and Pigeon IMU
     * Note: Do not create the Pigeon IMU in the first test
     */
    public SubDriveTrain(int leftFront, int leftRear, int rightFront, int rightRear, int gyro1) {
        /*
         * 1. Make instances of all 4 WPI_TalonFX motor classes
         * 2. Factory reset each
         * 3. Set rear motors to follow the front motors
         * 4. Set both right motors to inverted (run in reverse from the left side)
         * 5. Set neutral mode to coast all four motors
         * 6. Set the feedback sensor for both front motors
         * 7. Set sensor phase for both front motors
         * 8. Set nominal and peak output for both front motors
         * 9. Set FF, PG, IG, DG for velocity control for both front motors
         */
        this.LEFT_FRONT = new WPI_TalonFX(leftFront);
        this.LEFT_REAR = new WPI_TalonFX(leftRear);
        this.RIGHT_FRONT = new WPI_TalonFX(rightFront);
        this.RIGHT_REAR = new WPI_TalonFX(rightRear);
        this.GYROSCOPE = new WPI_PigeonIMU(gyro1);
        this.MOTOR5 = new WPI_TalonFX(5);
        // AUTODRIVE = new DifferentialDrive(LEFT_MOTORS, RIGHT_MOTORS);
        this.MOTOR5.setNeutralMode(NeutralMode.Brake);
        // Kinematics
        // Kinematics translates drivetrain linear and angular speed to left / right
        // wheel speeds
        // To do this translation it needs to know the drive train "width"
        this.KINEMATICS = new DifferentialDriveKinematics(
                TechnicalConstants.DRIVE_TRAIN_WIDTH);

        // Odometry
        // Odometry gives an estimate of the robot "pose" - which is the x,z coords plus
        // heading based on the gyroscope and left/right encoders
        this.ODOMETRY = new DifferentialDrivePoseEstimator(
                this.KINEMATICS,
                Rotation2d.fromDegrees(0),
                this.getLeftEncoderMeters(),
                this.getRightEncoderMeters(),
                new Pose2d());

        this.LEFT_FRONT.configFactoryDefault();
        this.LEFT_REAR.configFactoryDefault();
        this.RIGHT_FRONT.configFactoryDefault();
        this.RIGHT_REAR.configFactoryDefault();
        this.GYROSCOPE.reset();

        this.LEFT_REAR.follow(this.LEFT_FRONT); // slave back to front motors
        this.RIGHT_REAR.follow(this.RIGHT_FRONT);

        this.LEFT_FRONT.setSensorPhase(false); // set sides sensor phase
        this.RIGHT_FRONT.setSensorPhase(false);

        this.LEFT_FRONT.setInverted(false); // invert right side
        this.LEFT_REAR.setInverted(false);
        this.RIGHT_FRONT.setInverted(true);
        this.RIGHT_REAR.setInverted(true);

        this.LEFT_FRONT.setNeutralMode(NeutralMode.Coast);
        this.LEFT_REAR.setNeutralMode(NeutralMode.Coast);
        this.RIGHT_FRONT.setNeutralMode(NeutralMode.Coast);
        this.RIGHT_REAR.setNeutralMode(NeutralMode.Coast);

        this.LEFT_FRONT.configClosedloopRamp(DT_Settings.RAMP_RATE_SECS);
        this.RIGHT_FRONT.configClosedloopRamp(DT_Settings.RAMP_RATE_SECS);

        /* Config sensor used for Primary PID [Velocity] */
        this.LEFT_FRONT.configSelectedFeedbackSensor(
                FeedbackDevice.IntegratedSensor,
                DT_PIDF.PID_LOOP_INDEX,
                DT_PIDF.TIMEOUT_MS);

        this.RIGHT_FRONT.configSelectedFeedbackSensor(
                FeedbackDevice.IntegratedSensor,
                DT_PIDF.PID_LOOP_INDEX,
                DT_PIDF.TIMEOUT_MS);

        /* Config the peak and nominal outputs */
        this.LEFT_FRONT.configNominalOutputForward(0, DT_PIDF.TIMEOUT_MS);
        this.LEFT_FRONT.configNominalOutputReverse(0, DT_PIDF.TIMEOUT_MS);
        this.LEFT_FRONT.configPeakOutputForward(1, DT_PIDF.TIMEOUT_MS);
        this.LEFT_FRONT.configPeakOutputReverse(-1, DT_PIDF.TIMEOUT_MS);

        this.RIGHT_FRONT.configNominalOutputForward(0, DT_PIDF.TIMEOUT_MS);
        this.RIGHT_FRONT.configNominalOutputReverse(0, DT_PIDF.TIMEOUT_MS);
        this.RIGHT_FRONT.configPeakOutputForward(1, DT_PIDF.TIMEOUT_MS);
        this.RIGHT_FRONT.configPeakOutputReverse(-1, DT_PIDF.TIMEOUT_MS);

        this.LEFT_FRONT.configMotionAcceleration(DT_Settings.MAX_VELOCITY);
        this.LEFT_FRONT.configMotionCruiseVelocity(DT_Settings.MM_CRUISECONTROL);

        this.RIGHT_FRONT.configMotionAcceleration(DT_Settings.MAX_VELOCITY);
        // this.RIGHT_FRONT.configMotionCruiseVelocity(DT_Set.DT_CRUISE_VEL_ENC);

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

        // MOTION MAGIC
        this.LEFT_FRONT.configMotionAcceleration(DT_Settings.MM_ACCELERATION);
        this.RIGHT_FRONT.configMotionAcceleration(DT_Settings.MM_ACCELERATION);
        this.LEFT_FRONT.configMotionCruiseVelocity(DT_Settings.MM_CRUISECONTROL);
        this.RIGHT_FRONT.configMotionCruiseVelocity(DT_Settings.MM_CRUISECONTROL);
    }

    /*
     * runCurveDrive
     * Method to run drive train in velocity control mode using "curve" arcade
     * Curve arcade sets the turn proportional to the forward/back joystick
     * ... therefore when forward is zero, turn is zero
     * This is the default drive method in tele-op periodic
     */
    public void runDrive(double forward, double rotate, boolean quickTurningEnabled) {
        double fward = Math.abs(forward);
        double sens = DT_Settings.TURN_SENSITIVITY;
        if (quickTurningEnabled) {
            fward = 1;
            sens = DT_Settings.QUICK_TURN;
        }
        double left = -forward + rotate * fward * sens;
        double right = -forward - rotate * fward * sens;
        double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));
        if (maxMagnitude > 1) {
            left /= maxMagnitude;
            right /= maxMagnitude;
        }
        this.LEFT_FRONT.set(ControlMode.Velocity, left * DT_Settings.MAX_VELOCITY);
        this.RIGHT_FRONT.set(ControlMode.Velocity, right * DT_Settings.MAX_VELOCITY);
    }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     * @param linearVel  Linear velocity in m/s.
     * @param angularVel Angular velocity in rad/s.
     */
    public void drive(double linearVel, double angularVel) {
        // yVel is always 0 because we can only move one direction at a time (I think)
        DifferentialDriveWheelSpeeds wheelSpeeds = KINEMATICS.toWheelSpeeds(new ChassisSpeeds(
                linearVel, 0.0, angularVel));
        setWheelSpeeds(wheelSpeeds);
    }

    private void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
        setWheelSpeeds(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond);
    }

    private void setWheelSpeeds(double leftMeterRate, double rightMeterRate) {
        final double leftFeedforward = LEFT_FEED_FORWARD.calculate(leftMeterRate);
        final double rightFeedforward = RIGHT_FEED_FORWARD.calculate(rightMeterRate);

        final double leftOutput = LEFT_PID_CONTROLLER.calculate(getLeftEncoderMeterRate(), leftMeterRate);
        final double rightOutput = RIGHT_PID_CONTROLLER.calculate(getRightEncoderMeterRate(), rightMeterRate);

        double leftVolts = leftFeedforward + leftOutput;
        double rightVolts = rightFeedforward + rightOutput;

        LEFT_FRONT.set(ControlMode.PercentOutput, leftVolts / RobotController.getBatteryVoltage());
        RIGHT_FRONT.set(ControlMode.PercentOutput, rightVolts / RobotController.getBatteryVoltage());
    }

    double error;
    double adjust;

    // I believe error and adjust could be moved into this function but not sure
    public void rotateInPlace(boolean rotateButton, double rotateTo) {
        if (rotateButton) {
            double error = rotateTo - this.getGyroAngle();
            if (error > 1.0) {
                adjust = DT_PIDF.ROTATE_PG * error + DT_PIDF.ROTATE_FRICTION;
            } else if (error < 1.0) {
                adjust = DT_PIDF.ROTATE_PG * error - DT_PIDF.ROTATE_FRICTION;
            }
            this.runDrive(0, adjust, true);
        }
    }

    public void runMotionMagic(double targetDistanceL, double targetDistanceR) {
        this.LEFT_FRONT.set(ControlMode.MotionMagic, targetDistanceL);
        this.RIGHT_FRONT.set(ControlMode.MotionMagic, targetDistanceR);
        // at 6 seconds go back
    }

    /*
     * getter methods to display values on the dashboard
     * 1. Get left front speed
     * 2. Get right front speed
     */
    public double getLeftSpeed() {
        return this.LEFT_FRONT.getSelectedSensorVelocity(); // replace 0 with get sensor velocity
    }

    public double getRightSpeed() {
        return this.RIGHT_FRONT.getSelectedSensorVelocity(); // replace 0 with get sensor velocity
    }

    public double getLeftPosition() {
        return this.LEFT_FRONT.getSelectedSensorPosition(); // Left Odometer
    }

    public double getRightPosition() {
        return this.RIGHT_FRONT.getSelectedSensorPosition(); // Right Odometer
    }

    public double getGyroAngle() {
        return (this.GYROSCOPE.getAngle() % 360); // Gyroscope relative rotation
    }

    public double getGyroAngleAbsolute() {
        return this.GYROSCOPE.getAngle(); // Gyroscope absolute rotation
    }

    public Rotation2d getGyroRotation2d() {
        // We have to negate this for pathfollowing
        return Rotation2d.fromDegrees(-this.getGyroAngleAbsolute()); // Converts angle into Rotation2d type
    }

    public double getGyroPitch() {
        return this.GYROSCOPE.getPitch();
    }

    public double getGyroYaw() {
        return this.GYROSCOPE.getYaw();
    }

    public void resetEncoders() {
        this.LEFT_FRONT.setSelectedSensorPosition(0);
        this.LEFT_REAR.setSelectedSensorPosition(0);
        this.RIGHT_FRONT.setSelectedSensorPosition(0);
        this.RIGHT_REAR.setSelectedSensorPosition(0);
    }

    /**
     * Called within the Teleop Periodic method, updates the Odometry with
     * the GyroRotation2d, LeftEncoderMeters, and RightEncoderMeters
     * 
     * @param void
     * @returns void
     */
    public void updateOdometry() {
        this.ODOMETRY.update(
                this.getGyroRotation2d(),
                this.getLeftEncoderMeters(),
                this.getRightEncoderMeters());
    }

    public void resetGyro() {
        // NOTE: This doesn't reset the Pitch, we'll figure it out when Allred tells us
        // We're using setFusedHeading instead of setYaw because getAngle uses
        // fusedHeading
        // this.GYROSCOPE.setYaw(0, Constants.DT_PIDF.TIMEOUT_MS);
        this.GYROSCOPE.setFusedHeading(0, Constants.DT_PIDF.TIMEOUT_MS);
    }

    public void resetNavigation() {
        this.resetNavigation(new Pose2d());
    }

    public void resetNavigation(Pose2d pose) {
        this.resetOdometry(pose);
        this.resetGyro();
        // resetEncoders() called in resetOdometry
    }

    public void resetOdometry() {
        this.resetOdometry(new Pose2d());
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        // System.out.println("Pose Target: (" + pose.getX() + ", " + pose.getY() + ")"
        // +
        // "\n New Rotation: " + pose.getRotation().getDegrees());
        // Rotation2d currentRotation = getGyroRotation2d();
        this.ODOMETRY.resetPosition(pose.getRotation(), this.getLeftEncoderMeters(),
                this.getRightEncoderMeters(), pose);
        // Pose2d newCurrentPose = this.getPose2d();
        // System.out.println("Pose Result: (" + newCurrentPose.getX() + ", " +
        // newCurrentPose.getY() + ")"
        // + "\n New Rotation: " + newCurrentPose.getRotation().getDegrees());
    }

    public double getLeftEncoderPos() {
        return (this.LEFT_FRONT.getSelectedSensorPosition() + this.LEFT_REAR.getSelectedSensorPosition()) / 2.0;
    }

    public double getRightEncoderPos() {
        return (this.RIGHT_FRONT.getSelectedSensorPosition() + this.RIGHT_REAR.getSelectedSensorPosition()) / 2.0;
    }

    public double getLeftEncoderMeters() {
        return this.getLeftEncoderPos() * TechnicalConstants.METERS_PER_TICK;
    }

    public double getRightEncoderMeters() {
        return this.getRightEncoderPos() * TechnicalConstants.METERS_PER_TICK;
    }

    public double getLeftEncoderVel() {
        return (this.LEFT_FRONT.getSelectedSensorVelocity() + this.LEFT_REAR.getSelectedSensorVelocity()) / 2.0;
    }

    public double getRightEncoderVel() {
        return (this.RIGHT_FRONT.getSelectedSensorVelocity() + this.RIGHT_REAR.getSelectedSensorVelocity()) / 2.0;
    }

    public double getLeftEncoderMeterRate() {
        return this.getLeftEncoderVel() * TechnicalConstants.TICKS_PER_DECI_SECOND_TO_METERS_PER_SECOND;
    }

    public double getRightEncoderMeterRate() {
        return this.getRightEncoderVel() * TechnicalConstants.TICKS_PER_DECI_SECOND_TO_METERS_PER_SECOND;
    }

    public DifferentialDriveWheelSpeeds getDirectWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(this.getLeftEncoderMeterRate(), this.getRightEncoderMeterRate());
    }

    public Pose2d getPose2d() {
        return this.ODOMETRY.getEstimatedPosition();
    }

    /**
     * 
     * @return Pose2D x and y in double[]
     */
    public double[] getPose2dArray() {
        Pose2d estimatedPosition = this.getPose2d();
        double[] coordArray = new double[] {
                estimatedPosition.getX(),
                estimatedPosition.getY(),
        };
        return coordArray;
    }

    /**
     * 
     * @param desiredPose The position where the robot wants to go
     * @return ChassisSpeeds object with wanted x and y velocity, and rotation in
     *         radians
     */
    public ChassisSpeeds ramseteCalculate(Trajectory.State desiredPose) {
        return this.RAMSETE.calculate(this.getPose2d(), desiredPose);
    }

    public void balanceChargeStation() {
        final double pitch = this.getGyroPitch();
        final double sign = Math.signum(pitch);

        double left;
        double right;

        if (Math.abs(pitch) < 2) { // Between -2 and 2
            left = 0;
            right = 0;
        } else {
            left = DT_Settings.MAX_BALANCE_VELOCITY * (Math.abs(pitch) / 15);
            right = DT_Settings.MAX_BALANCE_VELOCITY * (Math.abs(pitch) / 15);

            // We technically don't need a MIN here due to the Math never actually reaching
            // it (1000) unless we get pitch = 2, at which point it'll go to 0.

            // Clamp between MAX and MIN with some math to support both positive and
            // negative
            left = sign * RobotMath.clamp(
                    Math.abs(left),
                    DT_Settings.MIN_BALANCE_VELOCITY,
                    DT_Settings.MAX_BALANCE_VELOCITY);

            right = sign * RobotMath.clamp(
                    Math.abs(right),
                    DT_Settings.MIN_BALANCE_VELOCITY,
                    DT_Settings.MAX_BALANCE_VELOCITY);
        }

        this.LEFT_FRONT.set(ControlMode.Velocity, left);
        this.RIGHT_FRONT.set(ControlMode.Velocity, right);
    }
}
