package frc.robot;

public class Constants {
    public static class Auto {
        // beta and zeta are largely robot independent
        public static final double BETA = 2.0;
        public static final double ZETA = 0.7;

        public static final double BALANCE_MAX_VELOCITY = 1625.0;

        public static final double BALANCE_PG = (1.0 / 15.0);
        public static final double BALANCE_IG = 0.0;
        public static final double BALANCE_DG = 0.0225;
    }

    // Addresses of all input / output devices
    public static class RioPortMaps {
        // Drive Train (CAN ID's)
        public static final int LEFT_BACK_DRIVETRAIN = 0;
        public static final int RIGHT_BACK_DRIVETRAIN = 1;
        public static final int LEFT_FRONT_DRIVETRAIN = 2;
        public static final int RIGHT_FRONT_DRIVETRAIN = 3;

        public static final int WHEEL_DROPDOWN_SOLENOID = 2;
        // Arm
        public static final int PIVOT = 4;
        public static final int EXTEND = 5;
        public static final int EXTENSION_BRAKE_SOLENOID = 4;
        public static final int CLAW_SOLENOID = 3;
        // Intake
        public static final int BOTTOM_INTAKE = 6;
        public static final int TOP_INTAKE = 7;
        public static final int INTAKE_LIMIT_SWITCH = 0;
        public static final int PLATTER = 8;
        public static final int INTAKE_SOLENOID_FORWARD = 0;
        public static final int INTAKE_SOLENOID_REVERSE = 1;
        // Gyro (CAN ID)
        public static final int GYRO = 0;
    }

    public static class PdpPortMaps {
        public static final int PIVOT = 2;
    }

    // Settings for arcade velocity drive
    public static class DT_Settings {
        public static final double TURN_SENSITIVITY = 0.65; // Joystick turn scaling factor for Curve Drive
        public static final double QUICK_TURN = 0.3; // Joystick turn scaling factor for QuickTurn Drive
        public static final double MAX_VELOCITY = 18_000.0; // Drive train max velocity encoder per 100ms
        public static final double MIN_BALANCE_VELOCITY = 2000.0; // Drive train min velocity when balancing
        public static final double BALANCE_MAX_VELOCITY = 2000.0; // Drive train max velocity when balancing
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

        // For Ramsette/pathing (Old Maverick)
        // public static final double STATIC_GAIN = 0.66877; // kS
        // public static final double VELOCITY_GAIN = 2.8809; // kV
        // public static final double ACCELERATION_GAIN = 0.23306; // kA

        // Maverick
        public static final double STATIC_GAIN = 0.21462; // kS
        public static final double VELOCITY_GAIN = 2.1823; // kV
        public static final double ACCELERATION_GAIN = 0.38106; // kA

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
    public static class Intake_Settings {
        // Rollers
        public static final double CUBE_INTAKE_SPEED = 0.4;
        public static final double CONE_INTAKE_SPEED = 0.525;
        public static final double REVERSE = -0.5;
        // Ripped docs from SupplyCurrentLimitConfiguration CTRE class
        /**
         * The "holding" current (amperes) to limit to when feature is activated.
         */
        public static final double INTAKE_CURRENT_LIMIT = 30.0;
        /**
         * Current must exceed this threshold (amperes) before limiting occurs.
         * If this value is less than currentLimit, then currentLimit is used as the
         * threshold.
         */
        public static final double INTAKE_CURRENT_THRESHOLD = 35.0;
        /**
         * How long current must exceed threshold (seconds) before limiting occurs.
         */
        public static final double INTAKE_CURRENT_THRESHOLD_TIME_SECONDS = 0.5;
        // Platter
        public static final double PLATTER_SPEED = 1_500; // equivalent to 0.1 in percentOutput
        public static final double PLATTER_FF = 1023 / 20_300.0;
        public static final double PLATTER_RAMP_RATE_SECS = 0.5;

        public static final int PLATTER_PID_LOOP_INDEX = 0;

        public static final int TIMEOUT_MS = 30;
    }

    // TODO: Don't forget about arm constants
    // Arm Settings
    public static class Arm_Settings {

        // PIVOT RUNS IN PercentOutput
        public static final double PIVOT_VELOCITY = 15.0; // degrees per second
        public static final double PIVOT_CRUISECONTROL = 15.0;

        public static final double PIVOT_MIN = -53.0; // Software limits degrees
        public static final double PIVOT_MAX = 180.0;

        public static final double PIVOT_RAMP_RATE_SECS = 0.5;
        public static final double PIVOT_MAX_VELOCITY = 0;

        public static final double PIVOT_DEGREES_OF_DTH_FORWARDS = 175.0; // degrees arm has to be retracted to minimum
        // extension
        public static final double PIVOT_DEGREES_OF_DTH_BACKWARDS = 5.0;

        public static final double PIVOT_OPERATOR_SPEED = 0.4;
        // EXTEND RUNS IN PERCENT OUTPUT
        public static final double EXTEND_ACCELERATION = 6000;
        public static final double EXTEND_CRUISECONTROL = 6000;
        public static final double EXTEND_MM_DTH_SLOWTO_PERCENT = 0.5;

        public static final double EXTEND_MAX = 45_000; // Software limits
        public static final double EXTEND_MIN = 5_000; // soft 0
        public static final double EXTEND_TUCKED = 17_000; // we use this in SubArm to check if the arm is retracted or
                                                           // not
        public static final double EXTEND_HARD_LIMIT = 0;

        public static final int TIMEOUT_MS = 30;

        public static final double EXTEND_OPERATOR_SPEED = 0.1;

        // Arm setpoints (degrees)
        public static final double PIVOT_HIGH = 160;
        public static final double EXTEND_HIGH = 43_100;

        public static final double PIVOT_MID = 180;
        public static final double EXTEND_MID = EXTEND_TUCKED;

        // public static final double PIVOT_LOW = 0;
        // public static final double EXTEND_LOW = 0;

        public static final double PIVOT_TRAVEL = 0;
        public static final double EXTEND_TRAVEL = 0;

        public static final double PIVOT_PICK_CONE = -54;
        public static final double EXTEND_PICK_CONE = 0;

        public static final double PIVOT_PICK_CUBE = -37;
        public static final double EXTEND_PICK_CUBE = 0;

        public static final double PIVOT_START = -40.0;
        public static final double EXTEND_START = 0;
    }

    // Arm PID
    public static class ARM_PIDF {
        public static final double EXTEND_PG = 0.3; // proportional
        public static final double EXTEND_IG = 0.0; // integral
        public static final double EXTEND_DG = 0.0; // derivative
        public static final double EXTEND_F = 1023.0 / 20300.0; // feedforward

        public static final double PIVOT_PG = 0.0; // proportional
        public static final double PIVOT_IG = 0.0; // integral
        public static final double PIVOT_DG = 0.0; // derivative
        public static final double PIVOT_F = 1023.0 / 20300.0; // feedforward

        public static final double PIVOT_SG = 0.0;
        public static final double PIVOT_GG = 0.0;
        public static final double PIVOT_VG = 0.0;
        public static final double PIVOT_AG = 0.0;

        public static final int PID_SLOT_INDEX = 0;
        public static final int PID_LOOP_INDEX = 0;
        public static final int TIMEOUT_MS = 30;
    }

    public static class TechnicalConstants {
        // GENERAL INFO
        public static final double TAU = 2 * Math.PI;
        public static final double INCHES_TO_METERS = 0.0254;
        public static final double MAX_FALCON_SPEED_ENC = 20_300.0;
        public static final double FALCON_TICKS = 2048.0;

        public static final double ARM_LENGTH_CONSTRAINT = 0.83; // arm at the maximum length

        // DRIVE TRAIN INFO
        public static final double DRIVE_TRAIN_GEAR_RATIO = 1 / 9.8;
        public static final double DRIVE_TRAIN_WIDTH = 0.7;
        // neither using manual or calculated
        // .56 meters, measured trackwidth manually is meter
        // INF calculated

        // ODOMETRY CONSTANTS
        // 2 * INCHES_TO_METER is wheel radius in meters
        public static final double METERS_PER_MOTOR_ROTATION = DRIVE_TRAIN_GEAR_RATIO * (3 * INCHES_TO_METERS) * TAU;
        public static final double METERS_PER_TICK = METERS_PER_MOTOR_ROTATION / FALCON_TICKS;
        /*
         * This constant is used to translate Falcon speed readings into encoder counts
         * per 100ms to meters per second
         */
        public static final double TICKS_PER_DECI_SECOND_TO_METERS_PER_SECOND = METERS_PER_MOTOR_ROTATION / FALCON_TICKS
                * 10;

        // ARM INFO
        public static final double PIVOT_GEARBOX_REDUCTION_RATIO = 64.0 / 1.0; // 64 to 1 reduction
        public static final double PIVOT_SPROCKET_REDUCTION_RATIO = 64.0 / 22.0; // 64 to 22 reduction
        public static final double PIVOT_GEAR_RATIO = PIVOT_GEARBOX_REDUCTION_RATIO *
                PIVOT_SPROCKET_REDUCTION_RATIO;

        public static final double ENCODER_COUNTS_PER_ROTATION = FALCON_TICKS * PIVOT_GEAR_RATIO;
        public static final double ENCODER_COUNTS_PER_DEGREE = ENCODER_COUNTS_PER_ROTATION / 360.0;
    }

    public static class Misc {
        public static final double CONTROLLER_DEADBAND = 0.1;
        public static final double CONTROLLER_POV_MARGIN = 20; // Margin left and right of POV bottons
    }
}
