// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class SubIntake {

    WPI_TalonFX INTAKE_TOP;
    WPI_TalonFX INTAKE_BOTTOM;
    WPI_TalonFX INTAKE_PLATTER;
    DoubleSolenoid INTAKE_SOLENOID;
    DigitalInput INTAKE_LIMIT_SWITCH;

    /**
     * @param intakeTop
     * @param intakeBottom
     * @param intakeSolenoidForward
     * @param intakeSolenoidReverse
     * @param intakePlatter
     */
    public SubIntake(int intakeBottom, int intakeTop, int intakeSolenoidForward, int intakeLimitSwitch,
            int intakeSolenoidReverse,
            int intakePlatter) {

        // Rollers
        this.INTAKE_BOTTOM = new WPI_TalonFX(intakeBottom);
        this.INTAKE_TOP = new WPI_TalonFX(intakeTop);

        this.INTAKE_BOTTOM.configFactoryDefault();
        this.INTAKE_TOP.configFactoryDefault();

        this.INTAKE_BOTTOM.setInverted(true);
        this.INTAKE_TOP.setInverted(true);

        this.INTAKE_BOTTOM.setNeutralMode(NeutralMode.Coast);
        this.INTAKE_TOP.setNeutralMode(NeutralMode.Coast);
        // Ok this work now
        this.INTAKE_BOTTOM.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(
                        true,
                        Constants.Intake_Settings.INTAKE_CURRENT_LIMIT,
                        Constants.Intake_Settings.INTAKE_CURRENT_THRESHOLD,
                        Constants.Intake_Settings.INTAKE_CURRENT_THRESHOLD_TIME_SECONDS));
        this.INTAKE_TOP.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(
                        true,
                        Constants.Intake_Settings.INTAKE_CURRENT_LIMIT,
                        Constants.Intake_Settings.INTAKE_CURRENT_THRESHOLD,
                        Constants.Intake_Settings.INTAKE_CURRENT_THRESHOLD_TIME_SECONDS));

        // Limit Switches
        this.INTAKE_LIMIT_SWITCH = new DigitalInput(intakeLimitSwitch);

        // Pneumatics
        this.INTAKE_SOLENOID = new DoubleSolenoid(
                PneumaticsModuleType.CTREPCM,
                intakeSolenoidForward,
                intakeSolenoidReverse);
        this.INTAKE_SOLENOID.set(DoubleSolenoid.Value.kReverse);

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
     * Runs intake at certain speed depending on `intakeMode`
     * 
     * @param intakeMode CONE, CUBE, CUBE_LIMITED, REVERSE
     */
    public void runIntake(SubIntakeModes intakeMode) {
        switch (intakeMode) {
            case CONE:
                this.INTAKE_BOTTOM.set(ControlMode.PercentOutput, Constants.Intake_Settings.INTAKE_SPEED);
                runPlatter(Constants.Intake_Settings.PLATTER_SPEED);
            case CUBE:
                this.INTAKE_TOP.set(ControlMode.PercentOutput, Constants.Intake_Settings.INTAKE_SPEED);
                break;
            case CUBE_LIMITED:
                if (this.getLimitSwitch()) {
                    stopIntake();
                } else {
                    this.INTAKE_TOP.set(ControlMode.PercentOutput, Constants.Intake_Settings.INTAKE_SPEED);
                }
                break;
            case REVERSE:
                INTAKE_BOTTOM.set(ControlMode.PercentOutput, Constants.Intake_Settings.REVERSE);
                INTAKE_TOP.set(ControlMode.PercentOutput, Constants.Intake_Settings.REVERSE);
                break;
            default:
                INTAKE_BOTTOM.set(ControlMode.PercentOutput, 0);
                INTAKE_TOP.set(ControlMode.PercentOutput, 0);
                System.out.println("ERROR:: invalid intakeMode");
                break;
        }
    }

    // #spin
    public void runPlatter(double velocity) {
        INTAKE_PLATTER.set(ControlMode.Velocity, velocity);
    }

    public void stopIntake() {
        INTAKE_BOTTOM.set(ControlMode.PercentOutput, 0.0);
        INTAKE_TOP.set(ControlMode.PercentOutput, 0.0);
    }

    public void stopPlatter() {
        INTAKE_PLATTER.set(ControlMode.Velocity, 0);
    }

    public void toggleIntakeSolenoid() {
        setIntakeSolenoid(!this.getIntakeDeployed());
    }

    public void setIntakeSolenoid(boolean deployIntake) {
        if (deployIntake) {
            this.INTAKE_SOLENOID.set(DoubleSolenoid.Value.kForward);
        } else if (!deployIntake) {
            this.INTAKE_SOLENOID.set(DoubleSolenoid.Value.kReverse);
        }
    }

    // getters
    public boolean getLimitSwitch() {
        return this.INTAKE_LIMIT_SWITCH.get();
    }

    public boolean getIntakeDeployed() {
        return INTAKE_SOLENOID.get() == DoubleSolenoid.Value.kForward;
    }

}
