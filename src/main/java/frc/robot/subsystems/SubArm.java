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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.Constants.ARM_PIDF;
import frc.robot.Constants.Arm_Settings;
import frc.robot.util.RobotMath;

/** Add your docs here. */
public class SubArm {

    private final Solenoid CLAW;
    private final WPI_TalonFX PIVOT;
    private final WPI_TalonFX EXTEND;
    private final Solenoid EXTEND_BRAKE;
    private final DigitalInput EXTEND_LIMIT_SWITCH;

    // PID CONTROLLERS
    private final ArmFeedforward PIVOT_FEEDFORWARD = new ArmFeedforward(
            Constants.ARM_PIDF.PIVOT_SG,
            Constants.ARM_PIDF.PIVOT_GG,
            Constants.ARM_PIDF.PIVOT_VG,
            Constants.ARM_PIDF.PIVOT_AG);
    private final PIDController PIVOT_FEEDBACK = new PIDController(
            Constants.ARM_PIDF.PIVOT_PG,
            Constants.ARM_PIDF.PIVOT_IG,
            Constants.ARM_PIDF.PIVOT_DG);

    public SubArm(int pivot, int extend, int extendBrake, int claw, int extendLimitSwitch) {
        this.PIVOT_FEEDBACK.setTolerance(RobotMath.deg2rad(5)); // decrease this later

        this.PIVOT = new WPI_TalonFX(pivot);
        this.EXTEND = new WPI_TalonFX(extend);
        this.EXTEND_LIMIT_SWITCH = new DigitalInput(extendLimitSwitch);
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
        // TODO: Figure out if we actually need this probably do
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

    // public void resetPivotController() {
    // // this.PIVOT_CONTROLLER.setSetpoint(0);
    // // this.PIVOT_CONTROLLER.setTolerance(0);
    // // this.PIVOT_CONTROLLER.reset();
    // }

    // /**
    // * @param setpoint
    // */
    // public void setPivotController(double setpoint) {
    // // this.PIVOT_CONTROLLER.setSetpoint(setpoint);
    // // this.PIVOT_CONTROLLER.reset();
    // }

    // public double getPivotControllerOutput() {
    // return RobotMath.clamp(
    // this.PIVOT_CONTROLLER.calculate(this.getPivotAngle()),
    // Constants.Arm_Settings.PIVOT_MAX_VELOCITY,
    // Constants.Arm_Settings.PIVOT_MIN_VELOCITY);
    // }

    /**
     * @param pivotDegrees
     * @param extendEncoderCounts
     */
    public void armTo(double pivotDegrees, double extendEncoderCounts) {
        // if goes throguh degrees of death AND arm is not tucked
        if (getPathPassedThroughDegreesOfDeath(pivotDegrees)) {
            if (getArmTucked()) {
                pivotTo(pivotDegrees);
            } else if (getExtendIsCloseToDth()) {
                tuckArm();
                stopPivot();
            } else {
                pivotTo(pivotDegrees);
                tuckArm();
            }
        } else {
            pivotTo(pivotDegrees);
            extendTo(extendEncoderCounts);
        }
    }

    // setpoints
    public void startPosition() {
        armTo(Constants.Arm_Settings.PIVOT_START, Constants.Arm_Settings.EXTEND_START);
    }

    public void travelPosition() {
        armTo(Constants.Arm_Settings.PIVOT_TRAVEL, Constants.Arm_Settings.EXTEND_TRAVEL);
    }

    public void scoreHigh() {
        armTo(Constants.Arm_Settings.PIVOT_HIGH, Constants.Arm_Settings.EXTEND_HIGH);
    }

    public void scoreMid() {
        armTo(Constants.Arm_Settings.PIVOT_MID, Constants.Arm_Settings.EXTEND_MID);
    }

    // TODO: Possibly we don't need this
    // public void scoreLow() {
    // armTo(Constants.Arm_Settings.PIVOT_LOW, Constants.Arm_Settings.EXTEND_LOW);
    // }

    public void grabCone() {
        armTo(Constants.Arm_Settings.PIVOT_PICK_CONE, Constants.Arm_Settings.EXTEND_PICK_CONE);
    }

    public void grabCube() {
        armTo(Constants.Arm_Settings.PIVOT_PICK_CUBE, Constants.Arm_Settings.EXTEND_PICK_CUBE);
    }

    // Pivot uses feedforward and feedback controller
    public void pivotTo(final double setpointDegrees) {
        this.PIVOT_FEEDBACK.setSetpoint(RobotMath.deg2rad(setpointDegrees));

        if (this.PIVOT_FEEDBACK.atSetpoint()) {
            this.stopPivot();
            return;
        }

        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/combining-feedforward-feedback.html#using-feedforward-components-with-pid
        final double feedforward = this.PIVOT_FEEDFORWARD.calculate(
                RobotMath.deg2rad(setpointDegrees),
                RobotMath.deg2rad(Constants.Arm_Settings.PIVOT_ACCELERATION));
        final double feedback = this.PIVOT_FEEDBACK.calculate(RobotMath.deg2rad(this.getPivotAngle()));
        final double velocity = feedforward + feedback;

        this.runPivot(velocity, true);
    }

    public void runPivot(double percentValue) {
        this.runPivot(percentValue, false);
    }

    public void runPivot(double percentValue, boolean override) {
        if (override) {
            this.setPivotSpeed(percentValue);
        }
        // TODO: update to be type safe and confirm units
        else if (this.PIVOT.getSelectedSensorPosition() <= Constants.Arm_Settings.PIVOT_MAX
                || this.PIVOT.getSelectedSensorPosition() >= Constants.Arm_Settings.PIVOT_MIN) {
            this.setPivotSpeed(percentValue);
        } else {
            this.stopPivot();
        }
    }

    private void setPivotSpeed(double voltage) {
        this.PIVOT.set(ControlMode.PercentOutput, voltage / RobotController.getBatteryVoltage());
        // SmartDashboard.putNumber("PIVOT SPEED%", voltage /
        // RobotController.getBatteryVoltage());
        // SmartDashboard.putNumber("THE UNITS", voltage);
        // SmartDashboard.putBoolean("At Setpoint", this.PIVOT_FEEDBACK.atSetpoint());
        // SmartDashboard.putNumber("Angle", this.getPivotAngle());
    }

    public void stopPivot() {
        this.setPivotSpeed(0);
    }

    private boolean pivotBreakMode = false;

    public void pivotToggleBreakMode() {
        if (pivotBreakMode) {
            this.PIVOT.setNeutralMode(NeutralMode.Coast);
        } else {
            this.PIVOT.setNeutralMode(NeutralMode.Brake);
        }
    }

    // Extension uses motionmagic.
    public void extendTo(double targetDistanceExtend) {

        double mmacc = Constants.Arm_Settings.EXTEND_ACCELERATION * Constants.Arm_Settings.EXTEND_MM_DTH_SLOWTO_PERCENT;
        double mmcc = Constants.Arm_Settings.EXTEND_CRUISECONTROL * Constants.Arm_Settings.EXTEND_MM_DTH_SLOWTO_PERCENT;

        if (getExtendIsCloseToDth()) {
            this.EXTEND.configMotionAcceleration(mmacc);
            this.EXTEND.configMotionCruiseVelocity(mmcc);
        } else {
            this.EXTEND.configMotionAcceleration(Constants.Arm_Settings.EXTEND_ACCELERATION);
            this.EXTEND.configMotionCruiseVelocity(Constants.Arm_Settings.EXTEND_CRUISECONTROL);
        }
        EXTEND.set(ControlMode.MotionMagic, targetDistanceExtend);
    }

    public void runExtend(double percentValue) {
        this.runExtend(percentValue, false);
    }

    public void runExtend(double percentValue, boolean override) {
        if (override) {
            EXTEND.set(ControlMode.PercentOutput, percentValue);
        } else if (getExtendIsOkay()) {
            EXTEND.set(ControlMode.PercentOutput, percentValue);
        } else { // If getExtendIsOkay is not okay meaning false
            stopExtend();
        }
    }

    public void stopExtend() {
        this.EXTEND.set(ControlMode.PercentOutput, 0);
    }

    public void tuckArm() {
        EXTEND.set(ControlMode.MotionMagic, Constants.Arm_Settings.EXTEND_MIN);
    }

    public void stopArm() {
        this.stopPivot();
        this.stopExtend();
    }

    public void toggleClaw() {
        this.CLAW.set(!this.getClawOpen());
    }

    public void openClaw() {
        this.CLAW.set(false);
    }

    public void closeClaw() {
        this.CLAW.set(true);
    }

    public void resetArmEncoders() {
        resetPivotDistance();
        resetExtendDistance();
    }

    public void resetPivotDistance() {
        this.PIVOT.setSelectedSensorPosition(0);
    }

    public void resetExtendDistance() {
        this.EXTEND.setSelectedSensorPosition(0);
    }

    // getters
    public boolean getClawOpen() {
        return this.CLAW.get();
    }

    public boolean getPivotAtSetpoint() {
        return this.PIVOT_FEEDBACK.atSetpoint();
    }

    public double getPivotPosition() {
        return this.PIVOT.getSelectedSensorPosition();
    }

    private double getPivotAngle() {
        return (this.PIVOT.getSelectedSensorPosition() / Constants.TechnicalConstants.ENCODER_COUNTS_PER_DEGREE) % 360;
    }

    public double getExtendPosition() {
        return this.EXTEND.getSelectedSensorPosition();
    }

    public boolean getPathPassedThroughDegreesOfDeath(double setpoint) {
        double s = getPivotAngle();
        double e = setpoint;
        double hr = Constants.Arm_Settings.PIVOT_DEGREES_OF_DTH_FORWARDS;
        double lr = Constants.Arm_Settings.PIVOT_DEGREES_OF_DTH_BACKWARDS;
        return (e < lr && lr < s) || (e < hr && hr < s) || (e > lr && lr > s) || (e > hr && hr > s);
    }

    public boolean getExtendIsCloseToDth() {
        return Math.abs(this.getPivotAngle() - Constants.Arm_Settings.PIVOT_DEGREES_OF_DTH_BACKWARDS) < 10
                || Math.abs(this.getPivotAngle() - Constants.Arm_Settings.PIVOT_DEGREES_OF_DTH_FORWARDS) < 10;
    }

    public boolean getExtendIsOkay() {
        return this.getPivotAngle() > Constants.Arm_Settings.PIVOT_DEGREES_OF_DTH_BACKWARDS
                && this.getPivotAngle() < Constants.Arm_Settings.PIVOT_DEGREES_OF_DTH_FORWARDS;
    }

    public boolean getArmTucked() {
        return this.getExtendPosition() <= Constants.Arm_Settings.EXTEND_TUCKED;
    }
    // TODO: current limit motor or monitor amps

}
