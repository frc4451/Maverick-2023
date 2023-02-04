// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.SubDriveTrain;
import frc.robot.util.Constants;

/** Add your docs here. */
public class RobotContainer {

    public static final SubDriveTrain driveTrain = new SubDriveTrain(Constants.RioPortMaps.DT_LEFT_FRONT,
            Constants.RioPortMaps.DT_LEFT_BACK, Constants.RioPortMaps.DT_RIGHT_FRONT,
            Constants.RioPortMaps.DT_RIGHT_BACK, Constants.RioPortMaps.GYRO);
    public static final SubArm arm = new SubArm();
    public static final Field2d field = new Field2d();
}
