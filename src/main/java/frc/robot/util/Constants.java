// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class Constants {

    public static class RioPortMaps {

        // Drive Train
        public static final int DT_LEFT_FRONT = 2;
        public static final int DT_LEFT_BACK = 0;
        public static final int DT_RIGHT_FRONT = 3;
        public static final int DT_RIGHT_BACK = 1;
        // Gyro(s)
        public static final int GYRO = 0;
        // Intake

    }

    public static class DT_Settings {
        public static final double TURN_SENSITIVITY = 0.65;
        public static final double QUICK_TURN_SENS = 0.6;
        public static final double MAX_VELOCITY = 6000.0;
        public static final double RAMP_RATE_SECS = 0.5;
        // Cruise/Acceleration = seconds
        public static final double MM_ACCELERATION = 6000.0; // Motion Magic Acceleration in one second
        public static final double MM_CRUISECONTROL = 6000.0; // per tenth of second
    }

    public static class DT_PIDF {
        public static final double LEFT_PG = 0.1; // proportional
        public static final double LEFT_IG = 0.0; // integral
        public static final double LEFT_DG = 0.0; // derivative
        public static final double LEFT_FF = 1023.0 / 20300.0; // feedforward

        public static final double RIGHT_PG = 0.1; // proportional
        public static final double RIGHT_IG = 0.0; // integral
        public static final double RIGHT_DG = 0.0; // derivative
        public static final double RIGHT_FF = 1023.0 / 20300.0; // feedforward
        // For Ramsette/pathing
        public static final double STATIC_GAIN = -1; // kS
        public static final double VELOCITY_GAIN = -1; // kV
        public static final double ACCELERATION_GAIN = -1; // kA

        /**
         * Which PID slot to pull gains from. Starting from 2018, you can choose from
         * 0, 1, 2, or 3. Only the first two (0 and 1) are visible in web-based
         * configuration.
         */
        public static final int SLOT_INDEX = 0;

        /**
         * Talon FX supports multiple (cascaded) PID loops. For
         * now we just want the primary one.
         */
        public static final int PID_LOOP_INDEX = 0;

        /**
         * Set to zero to skip waiting for confirmation, set to nonzero to wait and
         * report to DS if action fails.
         */
        public static final int TIMEOUT_MS = 30;
    }

    public static class Autonomous {
        public static final double BETA = -1;
        public static final double ZETA = -1;
    }

    public static class TechnicalConstants {
        // GENERAL INFO
        public static final double TAU = 2 * Math.PI;
        public static final double INCHES_TO_METERS = 0.0254;
        public static final double MAX_FALCON_SPEED_ENC = 20_300;
        public static final double FALCON_TICKS = 2048;

        // DT INFO
        public static final double DT_GEAR_RATIO = 0;
        public static final double DT_WIDTH = 0;
        // ODOMETRY CONSTANTS
        // 2 * INCHES_TO_METERS is wheel radius in meters
        public static final double METERS_PER_MOTOR_ROTATION = DT_GEAR_RATIO * (2 * INCHES_TO_METERS) * TAU;
        public static final double METERS_PER_TICK = METERS_PER_MOTOR_ROTATION / FALCON_TICKS;
        /**
         * This constant is used to translate Falcon speed readings into encoder counts
         * per 100ms to meters per second
         */
        public static final double TICKS_PER_DECI_SECOND_TO_METERS_PER_SECOND = METERS_PER_MOTOR_ROTATION / FALCON_TICKS
                * 10; // Auto format for some reason puts this on a new line.
    }

    public static class Misc {
        public static final double CONTROLLER_DEADBAND = 0.1;
    }
}
