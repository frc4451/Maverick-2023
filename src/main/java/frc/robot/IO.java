// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class IO {

    public static class Driver {
        private static final XboxController driver = new XboxController(0);

        // Joysticks
        public static double getLeftY() {
            return Math.pow(joyDeadband(driver.getLeftY()), 2) * Math.signum(driver.getLeftY());
        }

        public static double getRightX() {
            return Math.pow(joyDeadband(driver.getRightX()), 2) * Math.signum(driver.getRightX());
        }

        public static boolean getRightTrigger() {return driver.getRightTriggerAxis() > 0.1;}
        public static boolean getLeftTrigger() {return driver.getLeftTriggerAxis() > 0.1;}
        public static boolean getLeftBumper(){return driver.getLeftBumper();}
        public static boolean getRightBumper(){return driver.getRightBumper();}
        public static boolean getButtonA() {return driver.getAButton();}
        public static boolean getButtonB() {return driver.getBButton();}
        public static boolean getButtonX() {return driver.getXButton();}
        public static boolean getButtonY() {return driver.getYButton();}
    }

    public static class Operator {
        private static final XboxController operator = new XboxController(1);

        public static boolean getRightTrigger() {return operator.getRightTriggerAxis() > 0.1;}
        public static boolean getLeftTrigger() {return operator.getLeftTriggerAxis() > 0.1;}
        public static boolean getLeftBumper(){return operator.getLeftBumper();}
        public static boolean getRightBumper(){return operator.getRightBumper();}
        public static boolean getButtonA() {return operator.getAButton();}
        public static boolean getButtonB() {return operator.getBButton();}
        public static boolean getButtonX() {return operator.getXButton();}
        public static boolean getButtonY() {return operator.getYButton();}
    }

    /*
     * Joystick deadband method makes sure the joysticks are set to zero when
     * released
     */
    public static double joyDeadband(double stickValue) {
        if (stickValue <= Constants.Misc.CONTROLLER_DEADBAND && stickValue >= -Constants.Misc.CONTROLLER_DEADBAND) {
            return 0;
        } else {
            return stickValue;
        }
    }

    /*
     * Joystick square method to provide a better driver control
     */
    public static double joySquareValue(double rawValue) {
        return Math.signum(rawValue) * rawValue * rawValue;
    }
}
