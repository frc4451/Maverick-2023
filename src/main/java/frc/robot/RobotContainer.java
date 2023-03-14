// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.RioPortMaps;
import frc.robot.subsystems.SubArm;
import frc.robot.subsystems.SubDriveTrain;
import frc.robot.subsystems.SubIntake;

public class RobotContainer {

        public static final SubDriveTrain driveTrain = new SubDriveTrain(RioPortMaps.LEFT_FRONT_DRIVETRAIN,
                        RioPortMaps.LEFT_BACK_DRIVETRAIN, RioPortMaps.RIGHT_FRONT_DRIVETRAIN,
                        RioPortMaps.RIGHT_BACK_DRIVETRAIN, RioPortMaps.GYRO, RioPortMaps.WHEEL_DROPDOWN_SOLENOID);
        public static final SubArm arm = new SubArm(RioPortMaps.PIVOT,
                        RioPortMaps.EXTEND, RioPortMaps.EXTENSION_BRAKE_SOLENOID, RioPortMaps.CLAW_SOLENOID);
        public static final SubIntake intake = new SubIntake(RioPortMaps.BOTTOM_INTAKE, RioPortMaps.TOP_INTAKE,
                        RioPortMaps.INTAKE_LIMIT_SWITCH, RioPortMaps.INTAKE_SOLENOID_FORWARD,
                        RioPortMaps.INTAKE_SOLENOID_REVERSE, RioPortMaps.PLATTER);

        public static final Field2d field = new Field2d();

        public static final PowerDistribution pdp = new PowerDistribution();
}
