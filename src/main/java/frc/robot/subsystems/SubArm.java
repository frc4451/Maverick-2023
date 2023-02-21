// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ARM_PIDF;
import frc.robot.Constants.Arm_Settings;
import frc.robot.util.RobotMath;

/** Add your docs here. */
public class SubArm {

    private final WPI_TalonFX PIVOT;
    private final WPI_TalonFX EXTEND;
    private final Solenoid EXTEND_BRAKE;
    private final Solenoid CLAW;

    // PID CONTROLLER
    private final ArmFeedforward PIVOT_FEEDFORWARD = new ArmFeedforward(
            Constants.ARM_PIDF.PIVOT_SG,
            Constants.ARM_PIDF.PIVOT_GG,
            Constants.ARM_PIDF.PIVOT_VG,
            Constants.ARM_PIDF.PIVOT_AG);
    private final PIDController PIVOT_FEEDBACK = new PIDController(
            Constants.ARM_PIDF.PIVOT_PG,
            Constants.ARM_PIDF.PIVOT_IG,
            Constants.ARM_PIDF.PIVOT_DG);

    public SubArm(int pivot, int extend, int extendBrake, int claw) {
        this.PIVOT = new WPI_TalonFX(pivot);
        this.EXTEND = new WPI_TalonFX(extend);
        this.EXTEND_BRAKE = new Solenoid(PneumaticsModuleType.CTREPCM, extendBrake);
        this.CLAW = new Solenoid(PneumaticsModuleType.CTREPCM, claw);

        this.PIVOT.configFactoryDefault();
        this.EXTEND.configFactoryDefault();

        this.PIVOT.setNeutralMode(NeutralMode.Brake);
        this.EXTEND.setNeutralMode(NeutralMode.Brake);

        this.EXTEND.setInverted(true);
        // motion profile settings
        // this.PIVOT.configMotionCruiseVelocity(Arm_Settings.PIVOT_CRUISECONTROL);
        // this.PIVOT.configMotionAcceleration(Arm_Settings.PIVOT_ACCELERATION);
        this.EXTEND.configMotionCruiseVelocity(Arm_Settings.EXTEND_CRUISECONTROL);
        this.EXTEND.configMotionAcceleration(Arm_Settings.EXTEND_ACCELERATION);

        // used for velocity control
        this.PIVOT.configClosedloopRamp(Constants.Arm_Settings.PIVOT_RAMP_RATE_SECS);

        // Pivot PIDF
        this.PIVOT.configSelectedFeedbackSensor(
                FeedbackDevice.IntegratedSensor,
                ARM_PIDF.PID_LOOP_INDEX,
                ARM_PIDF.TIMEOUT_MS);
        this.PIVOT.config_kP(
                ARM_PIDF.PID_LOOP_INDEX,
                ARM_PIDF.PIVOT_PG,
                ARM_PIDF.TIMEOUT_MS);
        this.PIVOT.config_kI(
                ARM_PIDF.PID_LOOP_INDEX,
                ARM_PIDF.PIVOT_IG,
                ARM_PIDF.TIMEOUT_MS);
        this.PIVOT.config_kD(
                ARM_PIDF.PID_LOOP_INDEX,
                ARM_PIDF.PIVOT_DG,
                ARM_PIDF.TIMEOUT_MS);
        this.PIVOT.config_kF(
                ARM_PIDF.PID_LOOP_INDEX,
                ARM_PIDF.PIVOT_F,
                ARM_PIDF.TIMEOUT_MS);

        this.EXTEND.configSelectedFeedbackSensor(
                FeedbackDevice.IntegratedSensor,
                ARM_PIDF.PID_LOOP_INDEX,
                ARM_PIDF.TIMEOUT_MS);
        this.EXTEND.config_kP(
                ARM_PIDF.PID_LOOP_INDEX,
                ARM_PIDF.EXTEND_PG,
                ARM_PIDF.TIMEOUT_MS);
        this.EXTEND.config_kI(
                ARM_PIDF.PID_LOOP_INDEX,
                ARM_PIDF.EXTEND_IG,
                ARM_PIDF.TIMEOUT_MS);
        this.EXTEND.config_kD(
                ARM_PIDF.PID_LOOP_INDEX,
                ARM_PIDF.EXTEND_DG,
                ARM_PIDF.TIMEOUT_MS);
        this.EXTEND.config_kF(
                ARM_PIDF.PID_LOOP_INDEX,
                ARM_PIDF.EXTEND_F,
                ARM_PIDF.TIMEOUT_MS);
    }

    public void resetPivotController() {
        // this.PIVOT_CONTROLLER.setSetpoint(0);
        // this.PIVOT_CONTROLLER.setTolerance(0);
        // this.PIVOT_CONTROLLER.reset();
    }

    /**
     * @param setpoint
     */
    public void setPivotController(double setpoint) {
        // this.PIVOT_CONTROLLER.setSetpoint(setpoint);
        // this.PIVOT_CONTROLLER.reset();
    }

    // public double getPivotControllerOutput() {
    // return RobotMath.clamp(
    // this.PIVOT_CONTROLLER.calculate(this.getPivotAngle()),
    // Constants.Arm_Settings.PIVOT_MAX_VELOCITY,
    // Constants.Arm_Settings.PIVOT_MIN_VELOCITY);
    // }

    public void pivotTo(final double setpoint) {
        this.PIVOT_FEEDBACK.setSetpoint(setpoint);

        if (this.PIVOT_FEEDBACK.atSetpoint()) {
            runArmPivot(0);
            return;
        }

        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/combining-feedforward-feedback.html#using-feedforward-components-with-pid
        double velocity = this.PIVOT_FEEDFORWARD.calculate(
                RobotMath.deg2rad(setpoint),
                Constants.Arm_Settings.PIVOT_ACCELERATION) + this.PIVOT_FEEDBACK.calculate(this.getPivotAngle());

        velocity = RobotMath.clamp(velocity, 0, Constants.Arm_Settings.PIVOT_MAX_VELOCITY);
        runArmPivot(velocity);
    }

    public void runArmPivot(double velocity) {
        this.PIVOT.set(ControlMode.PercentOutput, velocity);
    }

    public void runArmExtend(double percentValue) {
        this.runArmExtend(percentValue, false);
    }

    public void runArmExtend(double percentValue, boolean override) {
        if (override) {
            EXTEND.set(ControlMode.PercentOutput, percentValue);
        } else if ((Math.signum(percentValue) == 1) && (getExtendPosition() <= Constants.Arm_Settings.EXTEND_MAX)) {
            EXTEND.set(ControlMode.PercentOutput, percentValue);
        } else if (Math.signum(percentValue) == -1 && (getExtendPosition() >= Constants.Arm_Settings.EXTEND_MIN)) {
            EXTEND.set(ControlMode.PercentOutput, percentValue);
        }
    }

    public void runArmRetraction(double percentValue) {
        if (Math.signum(percentValue) == -1 && (getExtendPosition() >= Constants.Arm_Settings.EXTEND_MIN)) {
            EXTEND.set(ControlMode.PercentOutput, percentValue);
        } else {
            EXTEND.set(ControlMode.PercentOutput, 0.0);
        }
    }

    // public void runPivotMotionMagic(double targetDistancePivot) {
    // PIVOT.set(ControlMode.MotionMagic, targetDistancePivot);
    // }

    public void runExtendMotionMagic(double targetDistanceExtend) {
        EXTEND.set(ControlMode.MotionMagic, targetDistanceExtend);
    }

    public void stopArm() {
        this.runArmPivot(0);
        this.runArmExtend(0);
    }

    // getters
    public double getPivotPosition() {
        return PIVOT.getSelectedSensorPosition();
    }

    private double getPivotAngle() {
        return PIVOT.getSelectedSensorPosition() / Constants.TechnicalConstants.ENCODER_COUNTS_PER_DEGREE;
    }

    public double getExtendPosition() {
        return EXTEND.getSelectedSensorPosition();
    }

    public void resetPivotDistance() {
        PIVOT.setSelectedSensorPosition(0);
    }

    public void resetExtendDistance() {
        EXTEND.setSelectedSensorPosition(0);
    }
}
