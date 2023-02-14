// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;

public class SubIntake {

    WPI_TalonFX INTAKE_TOP;
    WPI_TalonFX INTAKE_BOTTOM;
    WPI_TalonFX INTAKE_PLATTER;
    DoubleSolenoid INTAKE_SOLENOID;

    /**
     * @param intakeTop
     * @param intakeBottom
     * @param intakeSolenoidForward
     * @param intakeSolenoidReverse
     * @param intakePlatter
     */
    public SubIntake(int intakeTop, int intakeBottom, int intakeSolenoidForward, int intakeSolenoidReverse,
            int intakePlatter) {

        // Rollers
        this.INTAKE_TOP = new WPI_TalonFX(intakeTop);
        this.INTAKE_BOTTOM = new WPI_TalonFX(intakeBottom);

        this.INTAKE_TOP.configFactoryDefault();
        this.INTAKE_BOTTOM.configFactoryDefault();

        this.INTAKE_TOP.setInverted(false);
        this.INTAKE_BOTTOM.setInverted(false);
        // TODO: Why this no work?!
        this.INTAKE_TOP.triggerThresholdCurrent(new SupplyCurrentLimitConfiguration(true,
                Constants.Intake_Settings.INTAKE_CURRENT_LIMIT, Constants.Intake_Settings.INTAKE_CURRENT_THRESHOLD,
                Constants.Intake_Settings.INTAKE_CURRENT_THRESHOLD_TIME_SECONDS));
        this.INTAKE_BOTTOM.triggerThresholdCurrent(new SupplyCurrentLimitConfiguration(true,
                Constants.Intake_Settings.INTAKE_CURRENT_LIMIT, Constants.Intake_Settings.INTAKE_CURRENT_THRESHOLD,
                Constants.Intake_Settings.INTAKE_CURRENT_THRESHOLD_TIME_SECONDS));

        // Pneumatics
        this.INTAKE_SOLENOID = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, intakeSolenoidForward,
                intakeSolenoidReverse);
        this.INTAKE_SOLENOID.set(Value.kReverse);
        // Platter
        this.INTAKE_PLATTER = new WPI_TalonFX(intakePlatter);

        this.INTAKE_PLATTER.configFactoryDefault();
        this.INTAKE_PLATTER.setInverted(false);
        this.INTAKE_PLATTER.setNeutralMode(NeutralMode.Brake);
        this.INTAKE_PLATTER.configClosedloopRamp(Constants.Intake_Settings.PLATTER_RAMP_RATE_SECS);

        this.INTAKE_PLATTER.configSelectedFeedbackSensor(
                FeedbackDevice.IntegratedSensor,
                Constants.Intake_Settings.PLATTER_PID_LOOP_INDEX,
                Constants.Intake_Settings.TIMEOUT_MS);
        this.INTAKE_PLATTER.config_kF(
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
                this.INTAKE_BOTTOM.set(ControlMode.PercentOutput, Constants.Intake_Settings.INTAKE_SPEED);
                runPlatter(true, Constants.Intake_Settings.PLATTER_SPEED);
            case "cube":
                this.INTAKE_TOP.set(ControlMode.PercentOutput, Constants.Intake_Settings.INTAKE_SPEED);
                break;
            case "reverse":
                INTAKE_BOTTOM.set(ControlMode.PercentOutput, Constants.Intake_Settings.REVERSE);
                INTAKE_TOP.set(ControlMode.PercentOutput, Constants.Intake_Settings.REVERSE);
                break;
            default:
                INTAKE_BOTTOM.set(ControlMode.PercentOutput, 0);
                INTAKE_TOP.set(ControlMode.PercentOutput, 0);
                System.out.println("invalid intakeMode");
                break;
        }
    }

    public void stopIntake() {
        INTAKE_TOP.set(ControlMode.PercentOutput, 0.0);
        INTAKE_BOTTOM.set(ControlMode.PercentOutput, 0.0);
    }

    public void runPlatter(boolean run, double velocity) {
        if (run) {
            INTAKE_PLATTER.set(ControlMode.Velocity, velocity);
        } else {
            INTAKE_PLATTER.set(ControlMode.Velocity, 0);
        }
    }

    boolean solenoidDeployed = false;

    public void toggleIntakeSolenoid() {
        if (solenoidDeployed) {
            setIntakeSolenoid(false);
            solenoidDeployed = false;
        } else {
            setIntakeSolenoid(true);
            solenoidDeployed = true;
        }
    }

    public void setIntakeSolenoid(boolean isDeployed) {
        if (isDeployed) {
            this.INTAKE_SOLENOID.set(Value.kForward);
            solenoidDeployed = true;
        } else if (!isDeployed) {
            this.INTAKE_SOLENOID.set(Value.kReverse);
            solenoidDeployed = false;
        }
    }
}
