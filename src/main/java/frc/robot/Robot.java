// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoContainer;
import frc.robot.trajectories.TrajectoryContainer;
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
  private static final String DEFAULT_AUTO = "Default";
  private static final String AUTO_ONE = "auto1";
  private static final String AUTO_TWO = "auto2";
  private static final String AUTO_THREE = "auto3";
  private String autoSelected;
  private final SendableChooser<String> chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    chooser.setDefaultOption("Default Auto", DEFAULT_AUTO);
    chooser.addOption("Auto 1", AUTO_ONE);
    chooser.addOption("Auto 2", AUTO_TWO);
    chooser.addOption("Auto 3", AUTO_THREE);
    SmartDashboard.putData("Auto choices", chooser);
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
    SmartDashboard.putNumber("Delta Speed Right-Left",
        RobotContainer.driveTrain.getRightSpeed() - RobotContainer.driveTrain.getLeftSpeed());
    SmartDashboard.putData("Field/Field", RobotContainer.field);
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
    RobotContainer.driveTrain.resetNavigation();

    autoSelected = chooser.getSelected();
    System.out.println("Auto selected: " + autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (autoSelected) {
      case AUTO_ONE:
        AutoContainer.auto1();
        break;
      case AUTO_TWO:
        AutoContainer.auto2();
        break;

      case DEFAULT_AUTO:
      default:
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    RobotContainer.driveTrain.resetGyro();
    RobotContainer.arm.resetArmEncoders();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    RobotContainer.driveTrain.runDrive(IO.Driver.getLeftY(), IO.Driver.getRightX(), IO.Driver.getLeftBumper());
    if (IO.Driver.getButtonB()) {
      RobotContainer.driveTrain.balanceChargeStation();
    }

    if ((IO.Driver.getRightY() > 0) || (IO.Driver.getRightY() <= 0)) {
      RobotContainer.arm.runArmExtend(-IO.Driver.getRightY(), IO.Driver.getRightStickButton());
    } else {
      RobotContainer.arm.runArmExtend(0.0, false);
    }
    if (IO.Driver.getButtonA()) {
      RobotContainer.arm.runExtendMotionMagic(100_000);
    } else if (IO.Driver.getButtonY()) {
      RobotContainer.arm.runExtendMotionMagic(Constants.ARM_Settings.EXTEND_MIN);
    }
    if (IO.Driver.getStartButton()) {
      RobotContainer.arm.resetArmEncoders();
    }
    RobotContainer.driveTrain.runDrive(IO.Driver.getLeftY(), IO.Driver.getRightX(), IO.Driver.getLeftBumper());

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    if (IO.Driver.getButtonY()) {
      switch (autoSelected) {
        case AUTO_ONE:
          RobotContainer.field.getObject("Trajectory 1").setTrajectory(TrajectoryContainer.trajectory1_1);
          RobotContainer.field.getObject("Trajectory 2").setTrajectory(TrajectoryContainer.trajectory1_2);
          RobotContainer.driveTrain.resetNavigation(TrajectoryContainer.trajectory1_1.getInitialPose());
          break;
        case AUTO_TWO:
          RobotContainer.field.getObject("Trajectory 1").setTrajectory(TrajectoryContainer.trajectory3);
          RobotContainer.field.getObject("Trajectory 2").setTrajectory(new Trajectory());
          RobotContainer.driveTrain.resetNavigation(TrajectoryContainer.trajectory3.getInitialPose());
          break;
        case AUTO_THREE:
          RobotContainer.field.getObject("Trajectory 1").setTrajectory(TrajectoryContainer.charge);
          RobotContainer.field.getObject("Trajectory 2").setTrajectory(new Trajectory());
          RobotContainer.driveTrain.resetNavigation(TrajectoryContainer.charge.getInitialPose());
          break;

        case DEFAULT_AUTO:
        default:
          RobotContainer.field.getObject("Trajectory 1").setTrajectory(new Trajectory());
          RobotContainer.field.getObject("Trajectory 2").setTrajectory(new Trajectory());
          RobotContainer.driveTrain.resetNavigation();
          break;
      }
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
