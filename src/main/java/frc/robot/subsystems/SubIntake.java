// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants;

public class SubIntake {

    WPI_VictorSPX INTAKE_TOP;
    WPI_VictorSPX INTAKE_BOTTOM;
    WPI_TalonFX INTAKE_PLATTER;

    public SubIntake(int intakeTop, int intakeBottom, int intakePlatter) {
        INTAKE_TOP = new WPI_VictorSPX(intakeTop);
        INTAKE_BOTTOM = new WPI_VictorSPX(intakeBottom);

        INTAKE_TOP.configFactoryDefault();
        INTAKE_BOTTOM.configFactoryDefault();

        INTAKE_TOP.setInverted(false);
        INTAKE_BOTTOM.setInverted(false);

        // Platter
        INTAKE_PLATTER = new WPI_TalonFX(intakePlatter);

        INTAKE_PLATTER.configFactoryDefault();
        INTAKE_PLATTER.setInverted(false);
        INTAKE_PLATTER.setNeutralMode(NeutralMode.Brake);
        INTAKE_PLATTER.configClosedloopRamp(Constants.Intake_Settings.PLATTER_RAMP_RATE_SECS);

        INTAKE_PLATTER.configSelectedFeedbackSensor(
                FeedbackDevice.IntegratedSensor,
                Constants.Intake_Settings.PLATTER_PID_LOOP_INDEX,
                Constants.Intake_Settings.TIMEOUT_MS);
        INTAKE_PLATTER.config_kF(
                Constants.Intake_Settings.PLATTER_PID_LOOP_INDEX,
                Constants.Intake_Settings.PLATTER_FF,
                Constants.Intake_Settings.TIMEOUT_MS);
    }

    /**
     * 
     * @param intakeMode
     *                   "cube", "cone", "reverse"
     */
    public void runIntake(String intakeMode) {
        switch (intakeMode) {
            case "cone":
                INTAKE_BOTTOM.set(ControlMode.PercentOutput, Constants.Intake_Settings.INTAKE_SPEED);
            case "cube":
                INTAKE_TOP.set(ControlMode.PercentOutput, Constants.Intake_Settings.INTAKE_SPEED);
                break;
            case "reverse":
                INTAKE_BOTTOM.set(ControlMode.PercentOutput, Constants.Intake_Settings.REVERSE);
                INTAKE_TOP.set(ControlMode.PercentOutput, Constants.Intake_Settings.REVERSE);
                break;
            default:
                break;
        }
    }

    public void stopIntake() {
        INTAKE_TOP.set(ControlMode.PercentOutput, 0.0);
        INTAKE_BOTTOM.set(ControlMode.PercentOutput, 0.0);
    }

    public void spinPlatter(double speed) {
        // INTAKE_PLATTER.set(ControlMode.PercentOutput, speed);
        INTAKE_PLATTER.set(ControlMode.Velocity, speed);
    }

    public void stopPlatter() {
        spinPlatter(0);
    }
}
