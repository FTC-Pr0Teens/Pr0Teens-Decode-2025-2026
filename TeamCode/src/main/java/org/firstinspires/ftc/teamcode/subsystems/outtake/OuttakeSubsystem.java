package org.firstinspires.ftc.teamcode.subsystems.outtake;

public class OuttakeSubsystem {
    public final static double DEG_TO_RAD = Math.PI / 180;

    /**
     * Computes the initial velocity of the ball.
     *
     * @param x     Horizontal distance
     * @param y     Vertical distance
     * @param theta Initial Angle relative to horizontal
     * @return initial velocity
     */
    public static double ballInitialVelocity(double x, double y, double theta) {
        double thetaRad = DEG_TO_RAD * theta;
        double numer = 9.8 * x * x;
        double denom1 = 2 * Math.cos(thetaRad) * Math.cos(thetaRad);
        double denom2 = x * Math.tan(thetaRad) - y;
        return Math.sqrt(numer / (denom1 * denom2));
    }

    public static double flywheelSpin(double v0, double radius) {
        return v0 / radius;
    }

    public static double flywheelSpin(double x, double y, double theta, double radius) {
        return ballInitialVelocity(x, y, theta) / radius;
    }





}


