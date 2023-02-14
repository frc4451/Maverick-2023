// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.Constants.ARM_PIDF;
import frc.robot.Constants.Arm_Settings;

/** Add your docs here. */
public class SubArm {

    private final WPI_TalonFX PIVOT;
    private final WPI_TalonFX EXTEND;
    private final Solenoid EXTEND_BRAKE;
    private final Solenoid CLAW;

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
        this.PIVOT.configMotionCruiseVelocity(Arm_Settings.PIVOT_CRUISECONTROL);
        this.PIVOT.configMotionAcceleration(Arm_Settings.PIVOT_ACCELERATION);
        this.EXTEND.configMotionCruiseVelocity(Arm_Settings.EXTEND_CRUISECONTROL);
        this.EXTEND.configMotionAcceleration(Arm_Settings.EXTEND_ACCELERATION);

        // MotionMagic PIDF
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

    // run PercentOutput
    public void runArmPivot(boolean run, double percentValue) {
        if (run) {
            PIVOT.set(ControlMode.PercentOutput, percentValue);
        } else {
            PIVOT.set(ControlMode.PercentOutput, 0);
        }
    }

    public void runArmExtend(boolean run, double percentValue, boolean override) {
        if (run) {
            if (override) {
                EXTEND.set(ControlMode.PercentOutput, percentValue);
            } else if ((Math.signum(percentValue) == 1) && (getExtendPosition() <= Constants.Arm_Settings.EXTEND_MAX)) {
                EXTEND.set(ControlMode.PercentOutput, percentValue);
            } else if (Math.signum(percentValue) == -1 && (getExtendPosition() >= Constants.Arm_Settings.EXTEND_MIN)) {
                EXTEND.set(ControlMode.PercentOutput, percentValue);
            }
        } else {
            EXTEND.set(ControlMode.PercentOutput, 0.0);
        }
    }

    public void runArmRetraction(double percentValue) {
        if (Math.signum(percentValue) == -1 && (getExtendPosition() >= Constants.Arm_Settings.EXTEND_MIN)) {
            EXTEND.set(ControlMode.PercentOutput, percentValue);
        } else {
            EXTEND.set(ControlMode.PercentOutput, 0.0);
        }
    }

    // run MotionMagic
    public void runPivotMotionMagic(double targetDistancePivot) {
        PIVOT.set(ControlMode.MotionMagic, targetDistancePivot);
    }

    public void runExtendMotionMagic(double targetDistanceExtend) {
        EXTEND.set(ControlMode.MotionMagic, targetDistanceExtend);
    }

    public void stopArm() {
        PIVOT.set(ControlMode.PercentOutput, 0);
        EXTEND.set(ControlMode.PercentOutput, 0);
    }

    // getters
    public double getPivotPosition() {
        return PIVOT.getSelectedSensorPosition();
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
