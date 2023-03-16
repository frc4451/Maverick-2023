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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ARM_PIDF;
import frc.robot.Constants.Arm_Settings;
import frc.robot.util.RobotMath;

/** Add your docs here. */
public class SubArm {

    private final Solenoid CLAW;
    private final WPI_TalonFX PIVOT;
    private final WPI_TalonFX EXTEND;
    private final Solenoid EXTEND_BRAKE_SOLENOID;
    private final Timer extendBrakeTimer = new Timer();

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

    public SubArm(int pivot, int extend, int extendBrake, int claw) {
        this.PIVOT_FEEDBACK.setTolerance(RobotMath.deg2rad(3)); // mess with this later

        this.PIVOT = new WPI_TalonFX(pivot);
        this.EXTEND = new WPI_TalonFX(extend);
        this.EXTEND_BRAKE_SOLENOID = new Solenoid(PneumaticsModuleType.CTREPCM, extendBrake);
        this.CLAW = new Solenoid(PneumaticsModuleType.CTREPCM, claw);

        this.PIVOT.configFactoryDefault();
        this.EXTEND.configFactoryDefault();

        this.PIVOT.setNeutralMode(NeutralMode.Brake);
        this.EXTEND.setNeutralMode(NeutralMode.Brake);

        this.EXTEND.setInverted(false);
        // motion profile settings
        // this.PIVOT.configMotionCruiseVelocity(Arm_Settings.PIVOT_CRUISECONTROL);
        // this.PIVOT.configMotionAcceleration(Arm_Settings.PIVOT_ACCELERATION);
        this.EXTEND.configMotionCruiseVelocity(Arm_Settings.EXTEND_CRUISECONTROL);
        this.EXTEND.configMotionAcceleration(Arm_Settings.EXTEND_ACCELERATION);

        // used for velocity control
        this.PIVOT.configClosedloopRamp(Constants.Arm_Settings.PIVOT_RAMP_RATE_SECS);

        // Pivot PIDF
        // TODO: Figure out if we actually need this probably do EDIT: WE DON'T
        // this.PIVOT.configSelectedFeedbackSensor(
        // FeedbackDevice.IntegratedSensor,
        // ARM_PIDF.PID_LOOP_INDEX,
        // ARM_PIDF.TIMEOUT_MS);
        // this.PIVOT.config_kP(
        // ARM_PIDF.PID_LOOP_INDEX,
        // ARM_PIDF.PIVOT_PG,
        // ARM_PIDF.TIMEOUT_MS);
        // this.PIVOT.config_kI(
        // ARM_PIDF.PID_LOOP_INDEX,
        // ARM_PIDF.PIVOT_IG,
        // ARM_PIDF.TIMEOUT_MS);
        // this.PIVOT.config_kD(
        // ARM_PIDF.PID_LOOP_INDEX,
        // ARM_PIDF.PIVOT_DG,
        // ARM_PIDF.TIMEOUT_MS);
        // this.PIVOT.config_kF(
        // ARM_PIDF.PID_LOOP_INDEX,
        // ARM_PIDF.PIVOT_F,
        // ARM_PIDF.TIMEOUT_MS);

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

    // OLD armTo
    // /**
    // * @param pivotDegrees
    // * @param extendEncoderCounts
    // */
    // public void armTo(double pivotDegrees, double extendEncoderCounts) {
    // // if goes throguh degrees of death AND arm is not tucked
    // // TODO: make this work with extendBrakeTimer
    // if (getPathPassedThroughDegreesOfDeath(pivotDegrees)) { // Get if the arm
    // passes through the top area
    // if (getArmTucked()) {
    // pivotTo(pivotDegrees);
    // } else if (getExtendIsCloseToDth()) { // Stop pivoting and tuck the arm if
    // it's close to the top area.
    // stopPivot();
    // tuckArm();
    // } else { //
    // pivotTo(pivotDegrees);
    // tuckArm();
    // }
    // } else { // If the arm is past the top, continue pivoting and extending
    // pivotTo(pivotDegrees);
    // extendTo(extendEncoderCounts);
    // }
    // }

    /**
     * Checks if the current path intersects with the tuck zone.
     * <br>
     * <p>
     * If it intersects the tuck zone:
     * <ul>
     * <li>Disallow pivoting until tucked.</li>
     * <li>Tuck arm.</li>
     * <li>Once tucked, pivot until outside tuck zone.</li>
     * </ul>
     * If it does NOT intersect the tuck zone:
     * <ul>
     * <li>Pivot and extend to respective setpoints.</li>
     * </ul>
     * 
     * @param pivotDegrees        Pivot setpoint in degrees.
     * @param extendEncoderCounts Extend setpoint in encoder counts.
     */

    public void armTo(double pivotDegrees, double extendEncoderCounts) {
        if (getShouldTuckToPivot(pivotDegrees)) { // If inside tuck zone
            if (!getArmTucked()) { // If arm is not tucked
                stopPivot(); // Disallow pivoting for armTo until tucked
                tuckArm(); // Tuck the arm if it is not tucked
            } else { // Once (if) arm is tucked
                pivotTo(pivotDegrees); // Pivot until does not intersect tuck zone
                stopExtend(); // Stop extending if it's tucked
            }
        } else { // Once (if) outside tuck zone
            // Go to setpoints
            pivotTo(pivotDegrees);
            extendTo(extendEncoderCounts);
        }
    }

    // setpoints
    public void startPosition() {
        // armTo(Constants.Arm_Settings.PIVOT_START,
        // Constants.Arm_Settings.EXTEND_START);
        // extendTo(Constants.Arm_Settings.EXTEND_START);
        pivotTo(Constants.Arm_Settings.PIVOT_START);

    }

    public void travelPosition() {
        // armTo(Constants.Arm_Settings.PIVOT_TRAVEL,
        // Constants.Arm_Settings.EXTEND_TRAVEL);
        // extendTo(Constants.Arm_Settings.EXTEND_TRAVEL);
        pivotTo(Constants.Arm_Settings.PIVOT_TRAVEL);

    }

    public void gotoHigh() {
        // armTo(Constants.Arm_Settings.PIVOT_HIGH, Constants.Arm_Settings.EXTEND_HIGH);
        // extendTo(Constants.Arm_Settings.EXTEND_HIGH);
        pivotTo(Constants.Arm_Settings.PIVOT_HIGH);
    }

    public void gotoMid() {
        // armTo(Constants.Arm_Settings.PIVOT_MID, Constants.Arm_Settings.EXTEND_MID);
        // extendTo(Constants.Arm_Settings.EXTEND_MID);
        pivotTo(Constants.Arm_Settings.PIVOT_MID);
    }

    // public void scoreLow() {
    // armTo(Constants.Arm_Settings.PIVOT_LOW, Constants.Arm_Settings.EXTEND_LOW);
    // }

    public void armPickCone() {
        // armTo(Constants.Arm_Settings.PIVOT_PICK_CONE,
        // Constants.Arm_Settings.EXTEND_PICK_CONE);
        // extendTo(Constants.Arm_Settings.EXTEND_PICK_CONE);
        pivotTo(Constants.Arm_Settings.PIVOT_PICK_CONE);

    }

    public void armPickCube() {
        // armTo(Constants.Arm_Settings.PIVOT_PICK_CUBE,
        // Constants.Arm_Settings.EXTEND_PICK_CUBE);
        // extendTo(Constants.Arm_Settings.EXTEND_PICK_CUBE);
        pivotTo(Constants.Arm_Settings.PIVOT_PICK_CUBE);

    }

    public double deg2encoder(double degrees) {
        return Constants.TechnicalConstants.ENCODER_COUNTS_PER_DEGREE * degrees;
    }

    public double encoder2deg(double clicks) {
        return clicks / Constants.TechnicalConstants.ENCODER_COUNTS_PER_DEGREE;
    }

    // Pivot uses feedforward and feedback controller
    public void pivotTo(final double setpointDegrees) {
        this.PIVOT_FEEDBACK.setSetpoint(RobotMath.deg2rad(setpointDegrees));

        if (this.PIVOT_FEEDBACK.atSetpoint()) {
            // this.PIVOT_FEEDBACK.reset();
            // this.stopPivot();
            // return;
            System.out.println("Setpoint");
        }

        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/combining-feedforward-feedback.html#using-feedforward-components-with-pid
        final double feedforward = this.PIVOT_FEEDFORWARD.calculate(
                RobotMath.deg2rad(setpointDegrees),
                RobotMath.deg2rad(Constants.Arm_Settings.PIVOT_VELOCITY));
        final double feedback = this.PIVOT_FEEDBACK.calculate(RobotMath.deg2rad(this.getPivotAngle()));
        final double velocity = feedforward + feedback;

        this.runPivot(velocity / RobotContainer.pdp.getVoltage(), true);
        System.out.println("velocity:" + velocity);
    }

    public void runPivot(double percentValue) {
        this.runPivot(percentValue, false);
    }

    public void runPivot(double percentValue, boolean override) {
        if (override) {
            this.PIVOT.set(ControlMode.PercentOutput, percentValue);
        } else if (this.encoder2deg(this.PIVOT.getSelectedSensorPosition()) <= Constants.Arm_Settings.PIVOT_MAX
                && Math.signum(percentValue) > 0) {
            this.PIVOT.set(ControlMode.PercentOutput, percentValue);
        } else if (this.encoder2deg(this.PIVOT.getSelectedSensorPosition()) >= Constants.Arm_Settings.PIVOT_MIN
                && Math.signum(percentValue) < 0) {
            this.PIVOT.set(ControlMode.PercentOutput, percentValue);
        } else {
            this.stopPivot();
        }
    }

    // private void setPivotSpeed(double voltage) {
    // this.PIVOT.set(ControlMode.PercentOutput, voltage /
    // RobotController.getBatteryVoltage());
    // }

    public void stopPivot() {
        this.PIVOT.set(ControlMode.PercentOutput, 0.0);
    }

    private boolean pivotBreakMode = true;

    public void pivotToggleBrakeMode() {
        if (pivotBreakMode) {
            this.PIVOT.setNeutralMode(NeutralMode.Coast);
        } else {
            this.PIVOT.setNeutralMode(NeutralMode.Brake);
        }
        pivotBreakMode = !pivotBreakMode;
    }

    // Extension using motionmagic.
    public void extendTo(double targetDistanceExtend) {
        // double accel = Constants.Arm_Settings.EXTEND_ACCELERATION *
        // Constants.Arm_Settings.EXTEND_MM_DTH_SLOWTO_PERCENT;
        // double cruiseAccel = Constants.Arm_Settings.EXTEND_CRUISECONTROL *
        // Constants.Arm_Settings.EXTEND_MM_DTH_SLOWTO_PERCENT;

        // Commented out degrees of death for now
        // if (getExtendIsCloseToDth()) {
        // this.EXTEND.configMotionAcceleration(accel);
        // this.EXTEND.configMotionCruiseVelocity(cruiseAccel);
        // } else {
        // }
        // setExtendIfTimer(ControlMode.MotionMagic, targetDistanceExtend);

        setExtendIfTimer(ControlMode.MotionMagic, targetDistanceExtend);
    }

    public void setExtendIfTimer(ControlMode mode, double value) {
        setArmBrakeOff();
        if (extendBrakeTimer()) {
            this.EXTEND.set(mode, value);
        }
    }

    public boolean extendBrakeTimer() {
        // start timer if reset
        if (this.extendBrakeTimer.get() == 0) {
            this.extendBrakeTimer.start();
        }
        // After time, stop timer, return true; else return false
        if (this.extendBrakeTimer.hasElapsed(0.2)) {
            this.extendBrakeTimer.stop();
            return true;
        } else {
            return false;
        }
    }

    public void runExtend(double percentValue) {
        this.runExtend(percentValue, false);
    }

    public void runExtend(double percentValue, boolean override) {
        if (override /* || getExtendIsOkay() */) {
            if (this.EXTEND.getSelectedSensorPosition() <= Constants.Arm_Settings.EXTEND_MAX
                    && Math.signum(percentValue) > 0) {
                setArmBrakeOff();
                if (extendBrakeTimer()) {
                    EXTEND.set(ControlMode.PercentOutput, percentValue);
                }
            } else if (this.EXTEND.getSelectedSensorPosition() >= Constants.Arm_Settings.EXTEND_MIN
                    && Math.signum(percentValue) < 0) {
                setArmBrakeOff();
                if (extendBrakeTimer()) {
                    EXTEND.set(ControlMode.PercentOutput, percentValue);
                }
            } else {
                stopExtend();
            }
        } else { // If getExtendIsOkay is not okay meaning false
            stopExtend();
        }
    }

    private void setArmBrakeOn() {
        this.EXTEND_BRAKE_SOLENOID.set(false);
    }

    private void setArmBrakeOff() {
        this.EXTEND_BRAKE_SOLENOID.set(true);
    }

    public void stopExtend() {
        setArmBrakeOn();
        this.EXTEND.set(ControlMode.PercentOutput, 0);
        this.extendBrakeTimer.reset();
    }

    public void tuckArm() {
        extendTo(Constants.Arm_Settings.EXTEND_MIN);
    }

    public void stopArm() {
        this.stopPivot();
        this.stopExtend();
    }

    public void toggleClaw() {
        this.CLAW.set(!this.getClawState());
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
        this.PIVOT.setSelectedSensorPosition(deg2encoder(Constants.Arm_Settings.PIVOT_START));
    }

    public void resetExtendDistance() {
        this.EXTEND.setSelectedSensorPosition(Constants.Arm_Settings.EXTEND_START);
    }

    public void setExtendBrakeSolenoid(boolean isEnabled) {
        this.EXTEND_BRAKE_SOLENOID.set(isEnabled);
    }

    public void toggleExtendBrakeSolenoid() {
        this.EXTEND_BRAKE_SOLENOID.set(!getExtendBrakeSolenoid());
    }

    // getters

    public boolean getExtendBrakeSolenoid() {
        return this.EXTEND_BRAKE_SOLENOID.get();
    }

    public boolean getClawState() {
        return this.CLAW.get();
    }

    public boolean getPivotAtSetpoint() {
        return this.PIVOT_FEEDBACK.atSetpoint();
    }

    public double getPivotPosition() {
        return this.PIVOT.getSelectedSensorPosition();
    }

    public double getPivotAngle() {
        return (this.PIVOT.getSelectedSensorPosition() / Constants.TechnicalConstants.ENCODER_COUNTS_PER_DEGREE) % 360;
    }

    public double getExtendPosition() {
        return this.EXTEND.getSelectedSensorPosition();
    }

    /**
     * Used for armTo method
     * 
     * @param setpoint
     * @return whether or not the top area intersects the path from current position
     *         to setpoint
     */
    public boolean getShouldTuckToPivot(double setpoint) {
        // Variable declarations | gets current angle and setpoint
        final double upperBound = Constants.Arm_Settings.PIVOT_SHOULD_TUCK_UPPER_BOUND;
        final double lowerBound = Constants.Arm_Settings.PIVOT_SHOULD_TUCK_LOWER_BOUND;
        double start = getPivotAngle();
        double end = setpoint;
        /*
         * Gets the max and min of start and end to simplify comparisons
         * Checks if the smaller number (from Math.min) is less than the upper bound
         * Checks if the bigger number (from Math.max) is more than the lower bound
         * Without using Math.max and Math.min, we would need checks for both of these:
         * 1. start is less than end
         * 2. end is less than start
         */
        return Math.min(start, end) < upperBound && Math.max(start, end) > lowerBound;
    }

    // public boolean getExtendIsCloseToDth() {
    // return Math.abs(this.getPivotAngle() -
    // Constants.Arm_Settings.PIVOT_DEGREES_OF_DTH_BACKWARDS) < 10
    // || Math.abs(this.getPivotAngle() -
    // Constants.Arm_Settings.PIVOT_DEGREES_OF_DTH_FORWARDS) < 10;
    // }

    // public boolean getExtendIsOkay() {
    // return this.getPivotAngle() >
    // Constants.Arm_Settings.PIVOT_DEGREES_OF_DTH_BACKWARDS
    // && this.getPivotAngle() <
    // Constants.Arm_Settings.PIVOT_DEGREES_OF_DTH_FORWARDS;
    // }

    public boolean getArmTucked() {
        return this.getExtendPosition() <= Constants.Arm_Settings.EXTEND_TUCKED;
    }

    public double getPivotAmps() {
        return RobotContainer.pdp.getCurrent(Constants.PdpPortMaps.PIVOT);
    }
}
