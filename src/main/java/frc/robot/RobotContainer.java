// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.RioPortMaps;
import frc.robot.subsystems.SubArm;
import frc.robot.subsystems.SubDriveTrain;

/** Add your docs here. */
public class RobotContainer {

        public static final SubDriveTrain driveTrain = new SubDriveTrain(Constants.RioPortMaps.LEFT_FRONT_DRIVETRAIN,
                        Constants.RioPortMaps.LEFT_BACK_DRIVETRAIN, Constants.RioPortMaps.RIGHT_FRONT_DRIVETRAIN,
                        Constants.RioPortMaps.RIGHT_BACK_DRIVETRAIN, Constants.RioPortMaps.GYRO);
        public static final SubArm arm = new SubArm(RioPortMaps.PIVOT,
                        Constants.RioPortMaps.EXTEND);

        public static final Field2d field = new Field2d();

        public static final PowerDistribution pdp = new PowerDistribution();
}
