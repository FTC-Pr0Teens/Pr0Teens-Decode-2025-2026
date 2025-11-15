package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.util.pidcore.PIDCore;


public class OuttakeSubsystem {
    private ElapsedTime timer;
    private PIDCore pidCore;
    private OuttakeConstants outtakeConstants;
    private double derivative = 0;
    private double error = 0;
    private double lastError = 0;
    private double integralSum = 0;
    private double timeChange = 0;
    private double errorChange = 0;
    public static double Kd = 0.0;
    public static double Kp = 0.8;
    public static double Ki = 0.00 ;
    private Hardware hw;


    // Output tracking
    private double outputPositionalValue = 0;


    // Control flags
    private boolean activateIntegral = false;

    // Safety limits
    private double integralLimit = 1000.0; // Prevent integral windup
    private double outputLimit = 1.0;      // Limit output magnitude
    // Ignore small errors

    // Time tracking
    private double lastTime = 0;
    public final static double DEG_TO_RAD = Math.PI / 180;
//    public double kd = 0.02;
//    public double kp = 0.01;
//    public double ki = 0.01;


    public OuttakeSubsystem(Hardware hw){
        pidCore = new PIDCore(OuttakeConstants.Kp, OuttakeConstants.Kd, OuttakeConstants.Ki);
        timer = new ElapsedTime();
        outtakeConstants = new OuttakeConstants();
        this.hw = hw;

    }
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
//
//    public static double flywheelSpin(double v0, double radius) {
//        return v0 / radius;
//    }
//
//    public static double flywheelSpin(double x, double y, double theta, double radius) {
//        return ballInitialVelocity(x, y, theta) / radius;
//    }

    public static double outPower(double v0, double radius){
        return (30 * v0) / (Math.PI * radius);
    }

    public double outputPositional(double targetRPM, double currentRPM) {
        error = targetRPM - currentRPM;

        double currentTime = timer.seconds();
        double deltaTime = currentTime - lastTime;
        if (deltaTime <= 0) deltaTime = 0.001;


        derivative = (error - lastError) / deltaTime;


        if (Math.abs(error) < 300 && Math.abs(integralSum) < integralLimit) {
            integralSum += error * deltaTime;
        } else {
            integralSum *= 0.95;
        }

        double maxRPM = 6000.0;
        double kF = 0.9;
        double feedForward = kF * (targetRPM / maxRPM);

        //don't need integral and derivative
        //outputPositionalValue = feedForward + (Kp * error) + (Kd * derivative) + (Ki * integralSum);
        outputPositionalValue = feedForward + (Kp * error);

        outputPositionalValue = Math.max(0.0, Math.min(outputLimit, outputPositionalValue));

        errorChange = error - lastError;
        lastError = error;
        lastTime = currentTime;

        return outputPositionalValue;
    }

    public double getDistance(double limelightHeight, double targetHeight, double limelightAngle, double ty) {
        double angleToTargetDeg = limelightAngle + ty;
        double angleToTargetRad = Math.toRadians(angleToTargetDeg);
        return (targetHeight - limelightHeight) / Math.tan(angleToTargetRad);
    }
    public double getTargetRPM(double distance) {
        double slope = 20;
        double baseRPM = 4000;
        return slope * distance + baseRPM;
    }
    public void aimAndShoot(DcMotor shooter, double distance, double currentRPM) {
        double targetRPM = getTargetRPM(distance);
        double outputPower = outputPositional(targetRPM, currentRPM);

        // Clip power to valid range
        outputPower = Math.max(Math.min(outputPower, 1.0), 0.0);

        shooter.setPower(outputPower);
    }

    public double getRPM(){
        return hw.shooter.getVelocity();
    }
}








