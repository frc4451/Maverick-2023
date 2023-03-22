// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class IO {

    // We could probably change this to instead inherit XboxController
    public static class Driver {
        private static final XboxController controller = new XboxController(0);

        // Joysticks
        public static double getLeftY() {
            return Math.pow(joyDeadband(controller.getLeftY()), 2) * Math.signum(controller.getLeftY());
        }

        public static double getRightY() {
            return Math.pow(joyDeadband(controller.getRightY()), 2) * Math.signum(controller.getLeftY());
        }

        public static double getRightX() {
            return Math.pow(joyDeadband(controller.getRightX()), 2) * Math.signum(controller.getRightX());
        }

        public static boolean getRightStickButton() {
            return controller.getRightStickButton();
        }

        public static boolean getRightTrigger() {
            return controller.getRightTriggerAxis() > 0.1;
        }

        public static boolean getLeftTrigger() {
            return controller.getLeftTriggerAxis() > 0.1;
        }

        public static boolean getLeftBumper() {
            return controller.getLeftBumper();
        }

        public static boolean getRightBumper() {
            return controller.getRightBumper();
        }

        public static boolean getButtonA() {
            return controller.getAButton();
        }

        public static boolean getButtonAPressed() {
            return controller.getAButtonPressed();
        }

        public static boolean getButtonB() {
            return controller.getBButton();
        }

        public static boolean getButtonBReleased() {
            return controller.getBButtonReleased();
        }

        public static boolean getButtonXPressed() {
            return controller.getXButtonPressed();
        }

        public static boolean getButtonY() {
            return controller.getYButton();
        }

        public static boolean getButtonYReleased() {
            return controller.getYButtonReleased();
        }

        public static boolean getPOVUp() {
            return (controller.getPOV() >= 0 && controller.getPOV() < Constants.Misc.CONTROLLER_POV_MARGIN)
                    || controller.getPOV() > 360 - Constants.Misc.CONTROLLER_POV_MARGIN;
        }

        public static boolean getPOVRight() {
            return controller.getPOV() < 90 + Constants.Misc.CONTROLLER_POV_MARGIN
                    && controller.getPOV() > 90 - Constants.Misc.CONTROLLER_POV_MARGIN;
        }

        public static boolean getPOVDown() {
            return controller.getPOV() < 180 + Constants.Misc.CONTROLLER_POV_MARGIN
                    && controller.getPOV() > 180 - Constants.Misc.CONTROLLER_POV_MARGIN;
        }

        public static boolean getPOVLeft() {
            return controller.getPOV() < 270 + Constants.Misc.CONTROLLER_POV_MARGIN
                    && controller.getPOV() > 270 - Constants.Misc.CONTROLLER_POV_MARGIN;
        }

        public static boolean getStartButtonPressed() {
            return controller.getStartButtonPressed();
        }
    }

    public static class Operator {
        private static final XboxController controller = new XboxController(1);

        // Joysticks
        public static double getLeftY() {
            return Math.pow(joyDeadband(controller.getLeftY()), 2) * Math.signum(controller.getLeftY());
        }

        public static double getRightY() {
            return Math.pow(joyDeadband(controller.getRightY()), 2) * Math.signum(controller.getRightY());
        }

        public static double getRightX() {
            return Math.pow(joyDeadband(controller.getRightX()), 2) * Math.signum(controller.getRightX());
        }

        public static boolean getRightStickButton() {
            return controller.getRightStickButton();
        }

        public static boolean getRightTrigger() {
            return controller.getRightTriggerAxis() > 0.1;
        }

        public static boolean getLeftTrigger() {
            return controller.getLeftTriggerAxis() > 0.1;
        }

        public static boolean getLeftBumper() {
            return controller.getLeftBumper();
        }

        public static boolean getRightBumper() {
            return controller.getRightBumper();
        }

        public static boolean getButtonAPressed() {
            return controller.getAButtonPressed();
        }

        public static boolean getButtonB() {
            return controller.getBButton();
        }

        public static boolean getButtonX() {
            return controller.getXButton();
        }

        public static boolean getButtonY() {
            return controller.getYButton();
        }

        public static boolean getPOVUp() {
            return (controller.getPOV() >= 0 && controller.getPOV() < Constants.Misc.CONTROLLER_POV_MARGIN)
                    || controller.getPOV() > 360 - Constants.Misc.CONTROLLER_POV_MARGIN;
        }

        public static boolean getPOVRight() {
            return controller.getPOV() < 90 + Constants.Misc.CONTROLLER_POV_MARGIN
                    && controller.getPOV() > 90 - Constants.Misc.CONTROLLER_POV_MARGIN;
        }

        public static boolean getPOVDown() {
            return controller.getPOV() < 180 + Constants.Misc.CONTROLLER_POV_MARGIN
                    && controller.getPOV() > 180 - Constants.Misc.CONTROLLER_POV_MARGIN;
        }

        public static boolean getPOVLeft() {
            return controller.getPOV() < 270 + Constants.Misc.CONTROLLER_POV_MARGIN
                    && controller.getPOV() > 270 - Constants.Misc.CONTROLLER_POV_MARGIN;
        }

        public static boolean getStartButtonPressed() {
            return controller.getStartButtonPressed();
        }
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

    // /*
    // * Joystick square method to provide a better driver control
    // */
    // public static double joySquareValue(double rawValue) {
    // return Math.signum(rawValue) * rawValue * rawValue;
    // }
}
