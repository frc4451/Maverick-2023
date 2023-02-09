/*
RobotMath is a container of public static math methods
 */

package frc.robot.util;

public class RobotMath {
    /**
     * @param value Value to clamp
     * @param min   Minimum value
     * @param max   Maximum value
     * @return value "clamped" between min and max
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * @param value Value to clamp
     * @param min   Minimum value
     * @param max   Maximum value
     * @return value "clamped" between min and max
     */
    public static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }
}