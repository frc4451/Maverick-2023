// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoContainer;
import frc.robot.subsystems.SubIntakeModes;
// import frc.robot.trajectories.TrajectoryContainer;
import frc.robot.auto.AutoStates;
import frc.robot.util.IO;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private AutoStates autoSelected;
    private final SendableChooser<AutoStates> chooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        RobotContainer.arm.resetArmEncoders();
        RobotContainer.arm.closeClaw();
        RobotContainer.driveTrain.resetNavigation();
        RobotContainer.driveTrain.resetBalanceController();
        RobotContainer.driveTrain.setCoastMode();

        chooser.setDefaultOption("Default", AutoStates.DEFAULT);
        for (AutoStates state : AutoStates.values()) {
            chooser.addOption(state.label, state);
        }

        SmartDashboard.putData("Auto Choices", chooser);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Update Odemetry
        RobotContainer.driveTrain.updateOdometry();
        // Update robot position on Field2d.
        RobotContainer.field.setRobotPose(RobotContainer.driveTrain.getPose2d());

        SmartDashboard.putNumber("Pose2D X", RobotContainer.driveTrain.getPose2d().getX());
        SmartDashboard.putNumber("Pose2D Y", RobotContainer.driveTrain.getPose2d().getY());
        SmartDashboard.putNumber("Pose2D Rotation", RobotContainer.driveTrain.getPose2d().getRotation().getDegrees());
        SmartDashboard.putNumber("Gyro Pitch", RobotContainer.driveTrain.getGyroPitch());
        SmartDashboard.putNumber("Gyro Rotation", RobotContainer.driveTrain.getGyroAngle());
        SmartDashboard.putNumber("Gyro Absolute Rotation", RobotContainer.driveTrain.getGyroAngleAbsolute());
        SmartDashboard.putNumber("Left Distance", RobotContainer.driveTrain.getLeftPosition());
        SmartDashboard.putNumber("Right Distance", RobotContainer.driveTrain.getRightPosition());
        SmartDashboard.putNumber("Left Speed", RobotContainer.driveTrain.getLeftSpeed());
        SmartDashboard.putNumber("Right Speed", RobotContainer.driveTrain.getRightSpeed());
        // SmartDashboard.putNumber(
        // "Delta Speed Right-Left",
        // RobotContainer.driveTrain.getRightSpeed() -
        // RobotContainer.driveTrain.getLeftSpeed());
        SmartDashboard.putData("Field/Field", RobotContainer.field);
        SmartDashboard.putBoolean("Pivot At Setpoint", RobotContainer.arm.getPivotAtSetpoint());
        SmartDashboard.putNumber("Amp/Pivot", RobotContainer.arm.getPivotAmps());
        SmartDashboard.putNumber("Pivot Position", RobotContainer.arm.getPivotPosition());
        SmartDashboard.putNumber("Pivot Degrees", RobotContainer.arm.getPivotAngle());
        SmartDashboard.putNumber("Balance Output", RobotContainer.driveTrain.getBalanceControllerOutput());
        SmartDashboard.putBoolean("Beam Break", RobotContainer.intake.getLimitSwitch());
        SmartDashboard.putBoolean("Intake Deployed", RobotContainer.intake.getIntakeDeployed());
        SmartDashboard.putBoolean("startbutton", IO.Operator.getStartButtonPressed());
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different
     * autonomous modes using the dashboard. The sendable chooser code works with
     * the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
     * chooser code and
     * uncomment the getString line to get the auto name from the text box below the
     * Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure
     * below with additional strings. If using the SendableChooser make sure to add
     * them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        RobotContainer.driveTrain.setCoastMode();
        RobotContainer.driveTrain.resetNavigation();
        RobotContainer.driveTrain.resetBalanceController();
        autoSelected = chooser.getSelected();
        // System.out.println("Auto Selected: " + autoSelected);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        autoSelected.routine.callback();
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        RobotContainer.driveTrain.setCoastMode();
        RobotContainer.driveTrain.resetGyro();
        RobotContainer.driveTrain.resetBalanceController();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // Misc

        if (IO.Operator.getStartButtonPressed()) {
            RobotContainer.arm.pivotToggleBrakeMode();
        }

        if (IO.Driver.getStartButtonPressed()) {
            RobotContainer.arm.toggleExtendBrakeSolenoid();
        }

        // DRIVER
        if (RobotContainer.intake.getIntakeDeployed()) {
            if (IO.Driver.getLeftTrigger()) {
                RobotContainer.intake.runIntake(SubIntakeModes.REVERSE);
            } else if (IO.Driver.getRightTrigger()) {
                RobotContainer.intake.runIntake(SubIntakeModes.CONE);
            } else if (IO.Driver.getRightBumper()) {
                RobotContainer.intake.runIntake(SubIntakeModes.CUBE_LIMITED);
            } else if (IO.Operator.getButtonB()) {
                RobotContainer.intake.runIntake(SubIntakeModes.CUBE);
            } else {
                RobotContainer.intake.stopIntake();
            }
        } else {
            RobotContainer.intake.stopIntake();
        }

        if (IO.Driver.getButtonXPressed()) {
            RobotContainer.intake.toggleIntakeSolenoid();
        }

        if (IO.Driver.getButtonA()) {
            RobotContainer.driveTrain.toggleDropdownWheels();
        }

        if (IO.Driver.getButtonB()) {
            RobotContainer.driveTrain.balanceChargeStation();
        } else {
            RobotContainer.driveTrain.runDrive(IO.Driver.getLeftY(), IO.Driver.getRightX(), IO.Driver.getLeftBumper());
        }

        // OPERATOR

        if (!(IO.Driver.getRightBumper() || IO.Operator.getButtonX() || IO.Operator.getButtonY())) {
            RobotContainer.intake.stopPlatter();
        } else {
            if (IO.Operator.getButtonX()) {
                RobotContainer.intake.runPlatter(Constants.Intake_Settings.PLATTER_SPEED);
            } else if (IO.Operator.getButtonY()) {
                RobotContainer.intake.runPlatter(-Constants.Intake_Settings.PLATTER_SPEED);
            }
        }

        if (IO.Operator.getButtonA()) {
            RobotContainer.arm.toggleClaw();
        }

        if (IO.Operator.getPOVUp()) {
            RobotContainer.arm.gotoHigh();
        } else if (IO.Operator.getPOVRight()) {
            RobotContainer.arm.gotoMid();
        } else if (IO.Operator.getPOVDown()) {
            RobotContainer.arm.armPickCone();
        } else if (IO.Operator.getPOVLeft()) {
            RobotContainer.arm.armPickCube();
        } else {
            // Manual arm control
            if (IO.Operator.getLeftTrigger()) {
                RobotContainer.arm.runPivot(Constants.Arm_Settings.PIVOT_OPERATOR_SPEED);
            } else if (IO.Operator.getRightTrigger()) {
                RobotContainer.arm.runPivot(-Constants.Arm_Settings.PIVOT_OPERATOR_SPEED);
            }

            if (IO.Operator.getLeftBumper()) {
                RobotContainer.arm.runExtend(Constants.Arm_Settings.EXTEND_OPERATOR_SPEED);
            } else if (IO.Operator.getRightBumper()) {
                RobotContainer.arm.runExtend(-Constants.Arm_Settings.EXTEND_OPERATOR_SPEED);
            }
        }
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        RobotContainer.arm.closeClaw();
        AutoContainer.resetAutoStep();
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
        // Update the autoSelected string even when disabled so we can use it
        autoSelected = chooser.getSelected();

        if (IO.Driver.getButtonB()) {
            RobotContainer.driveTrain.setBrakeMode();
        } else if (IO.Driver.getButtonA()) {
            RobotContainer.driveTrain.setCoastMode();
        }

        // If Y is pressed this'll set the field trajectories and reset the robot's
        // position to the trajectory's start
        if (IO.Driver.getButtonY()) {
            for (int i = 0; i < AutoStates.longestPathGroup; i++) {
                final Trajectory path;
                if (autoSelected.paths.get(i) != null) {
                    path = autoSelected.paths.get(i);
                } else {
                    path = new Trajectory();
                }
                RobotContainer.field.getObject("Trajectory " + i).setTrajectory(path);
            }
            RobotContainer.driveTrain.resetNavigation(autoSelected.paths.get(0).getInitialPose());
        }

        // Misc

        if (IO.Operator.getStartButtonPressed()) {
            RobotContainer.arm.pivotToggleBrakeMode();
        }

        if (IO.Driver.getStartButtonPressed()) {
            RobotContainer.arm.toggleExtendBrakeSolenoid();
        }
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
