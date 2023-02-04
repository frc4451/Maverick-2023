// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

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
import frc.robot.util.Constants;
import frc.robot.util.Constants.DT_PIDF;
import frc.robot.util.Constants.DT_Settings;
import frc.robot.util.Constants.TechnicalConstants;

/** Add your docs here. */
public class SubDriveTrain {

        private final WPI_TalonFX LEFT_FRONT;
        private final WPI_TalonFX LEFT_BACK;
        private final WPI_TalonFX RIGHT_FRONT;
        private final WPI_TalonFX RIGHT_BACK;

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
                        Constants.DT_PIDF.ACCELERATION_GAIN);

        private final SimpleMotorFeedforward RIGHT_FEED_FORWARD = new SimpleMotorFeedforward(
                        Constants.DT_PIDF.STATIC_GAIN,
                        Constants.DT_PIDF.VELOCITY_GAIN,
                        Constants.DT_PIDF.ACCELERATION_GAIN);

        /**
         * 
         * @param leftFront
         * @param leftBack
         * @param rightFront
         * @param rightBack
         * @param gyro
         */
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
                                new Pose2d());

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
                this.LEFT_FRONT.configClosedloopRamp(DT_Settings.RAMP_RATE_SECS);
                this.RIGHT_FRONT.configClosedloopRamp(DT_Settings.RAMP_RATE_SECS);
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
                double _forward = Math.abs(forward);
                double sensitivity = DT_Settings.TURN_SENSITIVITY;
                if (quickTurnEnabled) {
                        _forward = 1;
                        sensitivity = DT_Settings.QUICK_TURN_SENS;
                }
                double left = -forward + rotate * _forward * sensitivity;
                double right = -forward - rotate * _forward * sensitivity;
                double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));
                if (maxMagnitude > 1) {
                        left /= maxMagnitude;
                        right /= maxMagnitude;
                }
                this.LEFT_FRONT.set(ControlMode.Velocity, left * DT_Settings.MAX_VELOCITY);
                this.RIGHT_FRONT.set(ControlMode.Velocity, right * DT_Settings.MAX_VELOCITY);
        }

        public void runMotionMagic(double targetDistanceL, double targetDistanceR) {
                this.LEFT_FRONT.set(ControlMode.MotionMagic, targetDistanceL);
                this.RIGHT_FRONT.set(ControlMode.MotionMagic, targetDistanceR);
        }

        public void kinematicDrive(double linearVelocity, double angularVelocity) {
                DifferentialDriveWheelSpeeds wheelSpeeds = KINEMATICS
                                .toWheelSpeeds(new ChassisSpeeds(linearVelocity, 0.0, angularVelocity));
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
                return this.RIGHT_FRONT.getSelectedSensorPosition();
        }

        public double getGyroAngle() {
                return (this.GYROSCOPE.getAngle() % 360); // Gyroscope relative rotation
        }

        public double getGyroAngleAbsolute() {
                return this.GYROSCOPE.getAngle(); // Gyroscope absolute rotation
        }

        public double getGyroPitch() {
                return this.GYROSCOPE.getPitch();
        }

        public double getGyroYaw() {
                return this.GYROSCOPE.getYaw();
        }

        // Converts angle into Rotation2d type
        public Rotation2d getGyroRotation2d() {
                return Rotation2d.fromDegrees(this.getGyroAngleAbsolute());
        }

        public void resetGyro() {
                // reset() has a timeout of 0 whereas the below uses our defined one
                // GYROSCOPE.reset();
                this.GYROSCOPE.setYaw(0, Constants.DT_PIDF.TIMEOUT_MS);
        }

        public void resetEncoders() {
                this.LEFT_FRONT.setSelectedSensorPosition(0);
                this.LEFT_BACK.setSelectedSensorPosition(0);
                this.RIGHT_FRONT.setSelectedSensorPosition(0);
                this.LEFT_BACK.setSelectedSensorPosition(0);
        }

        // KINEMATICS/ODOMETRY/ETC..
        public void resetNavigation() {
                this.resetNavigation(new Pose2d());
        }

        public void resetNavigation(Pose2d pose) {
                this.resetGyro();
                this.resetEncoders();
                // resetEncoders() called in resetOdometry
        }

        public void resetOdometry() {
                this.resetOdometry(new Pose2d());
        }

        private void setWheelSpeeds() {

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

        public double getLeftEncoderPos() {
                return (this.LEFT_FRONT.getSelectedSensorPosition() + this.LEFT_REAR.getSelectedSensorPosition()) / 2.0;
        }

        public double getRightEncoderPos() {
                return (this.RIGHT_FRONT.getSelectedSensorPosition() + this.RIGHT_REAR.getSelectedSensorPosition())
                                / 2.0;
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
                return (this.RIGHT_FRONT.getSelectedSensorVelocity() + this.RIGHT_REAR.getSelectedSensorVelocity())
                                / 2.0;
        }

        public double getLeftEncoderMeterRate() {
                return this.getLeftEncoderVel() * TechnicalConstants.TICKS_PER_DECI_SECOND_TO_METERS_PER_SECOND;
        }

        public double getRightEncoderMeterRate() {
                return this.getRightEncoderVel() * TechnicalConstants.TICKS_PER_DECI_SECOND_TO_METERS_PER_SECOND;
        }

        public DifferentialDriveWheelSpeeds getDirectWheelSpeeds() {
                return new DifferentialDriveWheelSpeeds(this.getLeftEncoderMeterRate(),
                                this.getRightEncoderMeterRate());
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
}
