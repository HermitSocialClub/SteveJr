package org.firstinspires.ftc.teamcode.drive.opmode.Utils;

//stolen and adapted from kookybots
public class MathUtils {
    public static double clamp(double num, double min, double max) {
        return Math.max(min, Math.min(num, max));
    }

    public static double m (double degrees){
        return Math.toRadians(degrees);
    }
}