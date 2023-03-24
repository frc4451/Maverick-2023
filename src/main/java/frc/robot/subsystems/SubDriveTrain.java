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
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.RobotMath;

public class SubDriveTrain {
    // Class variable definitions. Define the variable names for the WPI_TalonFX.
    private final WPI_TalonFX LEFT_FRONT;
    private final WPI_TalonFX RIGHT_FRONT;
    private final WPI_TalonFX LEFT_REAR;
    private final WPI_TalonFX RIGHT_REAR;
    private final WPI_PigeonIMU GYROSCOPE;
    private final Solenoid DROPDOWN_SOLENOID;

    private final DifferentialDriveKinematics KINEMATICS;
    private final DifferentialDrivePoseEstimator ODOMETRY;

    private final RamseteController RAMSETE = new RamseteController(Constants.Auto.BETA, Constants.Auto.ZETA);

    private final PIDController BALANCE_CONTROLLER = new PIDController(
            Constants.Auto.BALANCE_PG,
            Constants.Auto.BALANCE_IG,
            Constants.Auto.BALANCE_DG);

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
    public SubDriveTrain(int leftFront, int leftRear, int rightFront, int rightRear, int gyro, int dropDownSolenoid) {
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
        this.GYROSCOPE = new WPI_PigeonIMU(gyro);
        this.DROPDOWN_SOLENOID = new Solenoid(PneumaticsModuleType.CTREPCM, dropDownSolenoid);

        // Kinematics translates drivetrain linear and angular speeds to left / right
        // wheel speeds
        // To do this translation it needs to know the drive train "width"
        this.KINEMATICS = new DifferentialDriveKinematics(
                Constants.TechnicalConstants.DRIVE_TRAIN_WIDTH);

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

        this.LEFT_FRONT.configClosedloopRamp(Constants.DT_Settings.RAMP_RATE_SECS);
        this.RIGHT_FRONT.configClosedloopRamp(Constants.DT_Settings.RAMP_RATE_SECS);

        /* Config sensor used for Primary PID [Velocity] */
        this.LEFT_FRONT.configSelectedFeedbackSensor(
                FeedbackDevice.IntegratedSensor,
                Constants.DT_PIDF.PID_LOOP_INDEX,
                Constants.DT_PIDF.TIMEOUT_MS);

        this.RIGHT_FRONT.configSelectedFeedbackSensor(
                FeedbackDevice.IntegratedSensor,
                Constants.DT_PIDF.PID_LOOP_INDEX,
                Constants.DT_PIDF.TIMEOUT_MS);

        /* Config the peak and nominal outputs */
        this.LEFT_FRONT.configNominalOutputForward(0, Constants.DT_PIDF.TIMEOUT_MS);
        this.LEFT_FRONT.configNominalOutputReverse(0, Constants.DT_PIDF.TIMEOUT_MS);
        this.LEFT_FRONT.configPeakOutputForward(1, Constants.DT_PIDF.TIMEOUT_MS);
        this.LEFT_FRONT.configPeakOutputReverse(-1, Constants.DT_PIDF.TIMEOUT_MS);

        this.RIGHT_FRONT.configNominalOutputForward(0, Constants.DT_PIDF.TIMEOUT_MS);
        this.RIGHT_FRONT.configNominalOutputReverse(0, Constants.DT_PIDF.TIMEOUT_MS);
        this.RIGHT_FRONT.configPeakOutputForward(1, Constants.DT_PIDF.TIMEOUT_MS);
        this.RIGHT_FRONT.configPeakOutputReverse(-1, Constants.DT_PIDF.TIMEOUT_MS);

        this.LEFT_FRONT.configMotionAcceleration(Constants.DT_Settings.MAX_VELOCITY);
        this.LEFT_FRONT.configMotionCruiseVelocity(Constants.DT_Settings.MM_CRUISECONTROL);

        this.RIGHT_FRONT.configMotionAcceleration(Constants.DT_Settings.MAX_VELOCITY);
        this.RIGHT_FRONT.configMotionCruiseVelocity(Constants.DT_Settings.MM_CRUISECONTROL);

        this.LEFT_FRONT.config_kP(
                Constants.DT_PIDF.PID_LOOP_INDEX,
                Constants.DT_PIDF.LEFT_PG,
                Constants.DT_PIDF.TIMEOUT_MS);
        this.LEFT_FRONT.config_kI(
                Constants.DT_PIDF.PID_LOOP_INDEX,
                Constants.DT_PIDF.LEFT_IG,
                Constants.DT_PIDF.TIMEOUT_MS);
        this.LEFT_FRONT.config_kD(
                Constants.DT_PIDF.PID_LOOP_INDEX,
                Constants.DT_PIDF.LEFT_DG,
                Constants.DT_PIDF.TIMEOUT_MS);
        this.LEFT_FRONT.config_kF(
                Constants.DT_PIDF.PID_LOOP_INDEX,
                Constants.DT_PIDF.LEFT_FF,
                Constants.DT_PIDF.TIMEOUT_MS);

        this.RIGHT_FRONT.config_kP(
                Constants.DT_PIDF.PID_LOOP_INDEX,
                Constants.DT_PIDF.RIGHT_PG,
                Constants.DT_PIDF.TIMEOUT_MS);
        this.RIGHT_FRONT.config_kI(
                Constants.DT_PIDF.PID_LOOP_INDEX,
                Constants.DT_PIDF.RIGHT_IG,
                Constants.DT_PIDF.TIMEOUT_MS);
        this.RIGHT_FRONT.config_kD(
                Constants.DT_PIDF.PID_LOOP_INDEX,
                Constants.DT_PIDF.RIGHT_DG,
                Constants.DT_PIDF.TIMEOUT_MS);
        this.RIGHT_FRONT.config_kF(
                Constants.DT_PIDF.PID_LOOP_INDEX,
                Constants.DT_PIDF.RIGHT_FF,
                Constants.DT_PIDF.TIMEOUT_MS);

        // MOTION MAGIC
        this.LEFT_FRONT.configMotionAcceleration(Constants.DT_Settings.MM_ACCELERATION);
        this.RIGHT_FRONT.configMotionAcceleration(Constants.DT_Settings.MM_ACCELERATION);
        this.LEFT_FRONT.configMotionCruiseVelocity(Constants.DT_Settings.MM_CRUISECONTROL);
        this.RIGHT_FRONT.configMotionCruiseVelocity(Constants.DT_Settings.MM_CRUISECONTROL);
    }

    /*
     * runCurveDrive
     * Method to run drive train in velocity control mode using "curve" arcade
     * Curve arcade sets the turn proportional to the forward/back joystick
     * ... therefore when forward is zero, turn is zero
     * This is the default drive method in tele-op periodic
     */
    public void runDrive(double forward, double rotate) {
        double _forward = Math.abs(forward);
        double sens = Constants.DT_Settings.TURN_SENSITIVITY;
        if (forward == 0) {
            _forward = 1;
            sens = Constants.DT_Settings.QUICK_TURN;
        }
        double left = -forward + rotate * Math.max(_forward * sens, Constants.DT_Settings.MIN_TURN);
        double right = -forward - rotate * Math.max(_forward * sens, Constants.DT_Settings.MIN_TURN);
        double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));
        if (maxMagnitude > 1) {
            left /= maxMagnitude;
            right /= maxMagnitude;
        }
        // System.out.println(sens);
        this.LEFT_FRONT.set(ControlMode.Velocity, left * Constants.DT_Settings.MAX_VELOCITY);
        this.RIGHT_FRONT.set(ControlMode.Velocity, right * Constants.DT_Settings.MAX_VELOCITY);
    }

    /**
     * The Following is experimental, please proceed with caution.
     * 
     * @param forward
     * @param rotate
     * @param quickTurningEnabled
     */
    // public void runDrive(double forward, double rotate, boolean
    // quickTurningEnabled) {
    // double _forward = Math.abs(forward);
    // double sens = Constants.DT_Settings.TURN_SENSITIVITY;
    // // This is experimental, proceed with caution.
    // if (quickTurningEnabled) {
    // _forward = 1;
    // sens = Constants.DT_Settings.QUICK_TURN;
    // }
    // double left = -forward + rotate * _forward * sens;
    // double right = -forward - rotate * _forward * sens;
    // double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));
    // if (maxMagnitude > 1) {
    // left /= maxMagnitude;
    // right /= maxMagnitude;
    // }
    // this.LEFT_FRONT.set(ControlMode.Velocity, left *
    // Constants.DT_Settings.MAX_VELOCITY);
    // this.RIGHT_FRONT.set(ControlMode.Velocity, right *
    // Constants.DT_Settings.MAX_VELOCITY);
    // }

    /**
     * Drives the robot with linear and angular velocities derived from a
     * ChassisSpeeds object
     * 
     * @param refChassisSpeeds
     */
    public void drive(ChassisSpeeds refChassisSpeeds) {
        this.setWheelSpeeds(this.KINEMATICS.toWheelSpeeds(refChassisSpeeds));
    }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     * @param linearVel  Linear velocity in m/s.
     * @param angularVel Angular velocity in rad/s.
     */
    public void drive(double linearVel, double angularVel) {
        ChassisSpeeds refChassisSpeeds = new ChassisSpeeds(linearVel, 0.0, angularVel);
        this.drive(refChassisSpeeds);
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

    public void toggleDropdownWheels() {
        this.DROPDOWN_SOLENOID.set(!getDropDownSolenoid());
    }

    // public void runMotionMagic(double targetDistanceL, double targetDistanceR) {
    // this.LEFT_FRONT.set(ControlMode.MotionMagic, targetDistanceL);
    // this.RIGHT_FRONT.set(ControlMode.MotionMagic, targetDistanceR);
    // // at 6 seconds go back
    // }

    // getters
    public boolean getDropDownSolenoid() {
        return this.DROPDOWN_SOLENOID.get();
    }

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
        this.resetGyro(0);
    }

    public void resetGyro(double angleDeg) {
        // NOTE: This doesn't reset the Pitch, we'll figure it out when Allred tells us
        // We're using setFusedHeading instead of setYaw because getAngle uses
        // fusedHeading
        // this.GYROSCOPE.setYaw(0, Constants.DT_PIDF.TIMEOUT_MS);
        // We did this funky math to make it set it correctly
        // if (DriverStation.getAlliance() == Alliance.Red && angleDeg == 0) {
        // angleDeg = 180;
        // }
        // this.GYROSCOPE.setFusedHeading((180.0 / 2.815) * angleDeg,
        // Constants.DT_PIDF.TIMEOUT_MS);
        this.GYROSCOPE.setFusedHeading((180.0 / 2.815) * angleDeg, Constants.DT_PIDF.TIMEOUT_MS);
    }

    public void resetNavigation() {
        this.resetNavigation(new Pose2d());
    }

    public void resetNavigation(Pose2d pose) {
        this.resetOdometry(pose);
        this.resetGyro(pose.getRotation().getDegrees());
        // resetEncoders() called in resetOdometry
    }

    public void resetOdometry() {
        this.resetOdometry(new Pose2d());
    }

    public void resetOdometry(Pose2d pose) {
        this.resetEncoders();
        this.ODOMETRY.resetPosition(
                pose.getRotation(),
                this.getLeftEncoderMeters(),
                this.getRightEncoderMeters(),
                pose);
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

    public void resetBalanceController() {
        this.BALANCE_CONTROLLER.setSetpoint(0); // We want our pitch at 0 degrees
        this.BALANCE_CONTROLLER.setTolerance(4); // We'll accept a pitch within degrees
        this.BALANCE_CONTROLLER.reset();
    }

    public double getBalanceControllerOutput() {
        return RobotMath.clamp(BALANCE_CONTROLLER.calculate(this.getGyroPitch()), -1, 1);
    }

    public void balanceChargeStation() {
        this.setBrakeMode();

        if (BALANCE_CONTROLLER.atSetpoint()) {
            this.drive(0, 0);
            return;
        }
        // if (Math.abs(this.getGyroPitch()) <
        // this.BALANCE_CONTROLLER.getPositionTolerance()) {
        // this.drive(0, 0);
        // return;
        // }

        final double speed = this.getBalanceControllerOutput() * Constants.Auto.BALANCE_MAX_VELOCITY;

        this.LEFT_FRONT.set(ControlMode.Velocity, speed);
        this.RIGHT_FRONT.set(ControlMode.Velocity, speed);
    }

    public void setCoastMode() {
        this.LEFT_FRONT.setNeutralMode(NeutralMode.Coast);
        this.LEFT_REAR.setNeutralMode(NeutralMode.Coast);
        this.RIGHT_FRONT.setNeutralMode(NeutralMode.Coast);
        this.RIGHT_REAR.setNeutralMode(NeutralMode.Coast);
    }

    public void setBrakeMode() {
        this.LEFT_FRONT.setNeutralMode(NeutralMode.Brake);
        this.LEFT_REAR.setNeutralMode(NeutralMode.Brake);
        this.RIGHT_FRONT.setNeutralMode(NeutralMode.Brake);
        this.RIGHT_REAR.setNeutralMode(NeutralMode.Brake);
    }

    public double getLeftEncoderPos() {
        return (this.LEFT_FRONT.getSelectedSensorPosition() + this.LEFT_REAR.getSelectedSensorPosition()) / 2.0;
    }

    public double getRightEncoderPos() {
        return (this.RIGHT_FRONT.getSelectedSensorPosition() + this.RIGHT_REAR.getSelectedSensorPosition()) / 2.0;
    }

    public double getLeftEncoderMeters() {
        return this.getLeftEncoderPos() * Constants.TechnicalConstants.METERS_PER_TICK;
    }

    public double getRightEncoderMeters() {
        return this.getRightEncoderPos() * Constants.TechnicalConstants.METERS_PER_TICK;
    }

    public double getLeftEncoderVel() {
        return (this.LEFT_FRONT.getSelectedSensorVelocity() + this.LEFT_REAR.getSelectedSensorVelocity()) / 2.0;
    }

    public double getRightEncoderVel() {
        return (this.RIGHT_FRONT.getSelectedSensorVelocity() + this.RIGHT_REAR.getSelectedSensorVelocity()) / 2.0;
    }

    public double getLeftEncoderMeterRate() {
        return this.getLeftEncoderVel() * Constants.TechnicalConstants.TICKS_PER_DECI_SECOND_TO_METERS_PER_SECOND;
    }

    public double getRightEncoderMeterRate() {
        return this.getRightEncoderVel() * Constants.TechnicalConstants.TICKS_PER_DECI_SECOND_TO_METERS_PER_SECOND;
    }
}