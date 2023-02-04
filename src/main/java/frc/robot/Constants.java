package frc.robot;

public class Constants {
    public static class Auto {
        // beta and zeta are largely robot independent
        public static final double BETA = 2.0;
        public static final double ZETA = 0.7;
    }

    // Addresses of all input / output devices
    public static class RioPortMaps {
        // Drive Train (CAN ID's)
        public static final int LEFT_BACK_DRIVETRAIN = 0;
        public static final int RIGHT_BACK_DRIVETRAIN = 1;
        public static final int LEFT_FRONT_DRIVETRAIN = 2;
        public static final int RIGHT_FRONT_DRIVETRAIN = 3;
        // Gyro (CAN ID)
        public static final int GYRO = 0;
        // Intake
        public static final int TOP_INTAKE = 6;
        public static final int BOTTOM_INTAKE = 7;
        // Arm
        public static final int PIVOT = 4;
        public static final int EXTEND = 5;
        public static final int ARM_SOLENOID = 0;
        // Intakes
        // public static final int FRONT_INTAKE_MOTOR = 0;
        // public static final int BACK_INTAKE_MOTOR = 1;
        // public static final int FRONT_INTAKE_SOLENOID = 7;
        // public static final int BACK_INTAKE_SOLENOID = 6;
    }

    // Settings for arcade velocity drive
    public static class DT_Settings {
        public static final double TURN_SENSITIVITY = 0.65; // Joystick turn scaling factor for Curve Drive
        public static final double QUICK_TURN = 0.6; // Joystick turn scaling factor for QuickTurn Drive
        public static final double MAX_VELOCITY = 6000.0; // Drive train max velocity encoder per 100ms
        public static final double MIN_BALANCE_VELOCITY = 1000.0; // Drive train min velocity when balancing
        public static final double MAX_BALANCE_VELOCITY = 6000.0; // Drive train max velocity when balancing
        public static final double RAMP_RATE_SECS = 0.5; // Drive train ramp rate in velocity control
        /*
         * NOTE: Ramp rate - in seconds - is the time it takes for the output to go from
         * 0% output to 100% output.
         * 
         * This ramp rate is applied after the control loop.
         * For the drive train, it will limit large amperage spikes to accelerate the
         * velocity loop.
         * 
         * The PIDF velocity control loop does not have an acceleration limiter since
         * most of the output is
         * controlled by the feed forward term. Ramp rate effectively limits
         * acceleration.
         */
        // encoder counts Auto#, Step

        // Cruise/acceleration = seconds
        public static final double MM_ACCELERATION = 6000.0; // in one second
        public static final double MM_CRUISECONTROL = 6000.0; // per tenth of second
        public static final double INTAKE_SPEED = -0.7; // Intake speed
        public static final double GYRO_FRIXION_DEADBAND = 0.5;
        public static final double INTAKE_CENTERWHEEL_SPEED = 0.4; // Intake Centering wheel speed
    }

    // Drive train PIDF tuning values
    public static class DT_PIDF {
        public static final double LEFT_PG = 0.1; // proportional
        public static final double LEFT_IG = 0.0; // integral
        public static final double LEFT_DG = 0.0; // derivative
        public static final double LEFT_FF = 1023.0 / 20300.0; // feedforward

        public static final double RIGHT_PG = 0.1; // proportional
        public static final double RIGHT_IG = 0.0; // integral
        public static final double RIGHT_DG = 0.0; // derivative
        public static final double RIGHT_FF = 1023.0 / 20300.0; // feedforward

        public static final double ROTATE_PG = 0.009; // 0.015
        public static final double ROTATE_FRICTION = 0.05;

        // For Ramsette/pathing
        public static final double STATIC_GAIN = 0.66877; // kS
        public static final double VELOCITY_GAIN = 2.8809; // kV
        public static final double ACCELERATION_GAIN = 0.23306; // kA

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

    // Intake Settings
    public static class IntakeSettings {
        public static final double CUBE_PERCENT = 0.35;
        public static final double CONE_PERCENT = 0.50;
        public static final double REVERSE = -0.75;
    }

    // Arm Settings
    public static class ARM_Settings {
        public static final double PIVOT_ACCELERATION = 5000.0;
        public static final double PIVOT_CRUISECONTROL = 5000.0;
        public static final double EXTEND_ACCELERATION = 15_000.0;
        public static final double EXTEND_CRUISECONTROL = 15_000.0;

        public static final double EXTEND_MAX = 200_000;
        public static final double EXTEND_MIN = 15_500; // soft 0
        public static final double EXTEND_HARD_LIMIT = 0;
    }

    // Arm PID
    public static class ARM_PIDF {
        public static final double PIVOT_PG = 0.3; // proportional
        public static final double PIVOT_IG = 0.0; // integral
        public static final double PIVOT_DG = 0.0; // derivative
        public static final double PIVOT_F = 1023.0 / 20300.0; // feedforward

        public static final double EXTEND_PG = 0.3; // proportional
        public static final double EXTEND_IG = 0.0; // integral
        public static final double EXTEND_DG = 0.0; // derivative
        public static final double EXTEND_F = 1023.0 / 20300.0; // feedforward

        public static final int ARM_PID_SLOT_INDEX = 0;
        public static final int ARM_PID_LOOP_INDEX = 0;
        public static final int ARM_TIMEOUT_MS = 30;

    }

    public static class TechnicalConstants {
        // GENERAL INFO
        public static final double TAU = 2 * Math.PI;
        public static final double INCHES_TO_METERS = 0.0254;
        public static final double MAX_FALCON_SPEED_ENC = 20_300;
        public static final double FALCON_TICKS = 2048;

        // DRIVE TRAIN INFO
        public static final double DRIVE_TRAIN_GEAR_RATIO = 72.0 / 625.0;
        public static final double DRIVE_TRAIN_WIDTH = 0.8;
        // neither using manual or calculated
        // .62 meters, measured trackwidth manually is meter
        // .69282 calculated

        // ODOMETRY CONSTANTS
        // 2 * INCHES_TO_METER is wheel radius in meters
        public static final double METERS_PER_MOTOR_ROTATION = DRIVE_TRAIN_GEAR_RATIO * (2 * INCHES_TO_METERS) * TAU;
        public static final double METERS_PER_TICK = METERS_PER_MOTOR_ROTATION / FALCON_TICKS;
        /*
         * This constant is used to translate Falcon speed readings into encoder counts
         * per 100ms to meters per second
         */
        public static final double TICKS_PER_DECI_SECOND_TO_METERS_PER_SECOND = METERS_PER_MOTOR_ROTATION / FALCON_TICKS * 10;
    }

    public static class Misc {
        public static final double CONTROLLER_DEADBAND = 0.1;
    }
}
