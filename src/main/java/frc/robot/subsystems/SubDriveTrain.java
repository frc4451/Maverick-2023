// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.robot.Constants;

/** Add your docs here. */
public class SubDriveTrain {

    private final WPI_TalonFX LEFT_BACK;
    private final WPI_TalonFX RIGHT_BACK;
    private final WPI_TalonFX LEFT_FRONT;
    private final WPI_TalonFX RIGHT_FRONT;

    // private final WPI_GYRO GYROSCOPE;
    private final DifferentialDriveKinematics KINEMATICS;
    private final DifferentialDrivePoseEstimator ODOMETRY;

    private final RamseteController RAMSETE = new RamseteController(Constants.Autonomous.BETA,
            Constants.Autonomous.ZETA);
}
