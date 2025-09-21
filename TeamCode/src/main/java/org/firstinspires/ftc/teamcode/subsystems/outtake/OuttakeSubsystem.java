package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumConstants;
import org.firstinspires.ftc.teamcode.util.pidcore.PIDCore;
import static org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeConstants.*;


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


    public OuttakeSubsystem(){
        pidCore = new PIDCore(OuttakeConstants.Kp, OuttakeConstants.Kd, OuttakeConstants.Ki);
        timer = new ElapsedTime();

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

    public double outputPositional(double targetRPM, double curretRPM) {

     error = targetRPM - curretRPM;



        double currentTime = timer.seconds();
        double deltaTime = currentTime - lastTime;
        if (deltaTime <= 0) {
            deltaTime = 0.001;
        }


        derivative = (error - lastError) / deltaTime;


        if (activateIntegral && Math.abs(integralSum) < integralLimit) {
            integralSum += error * deltaTime;

            if (Math.abs(integralSum) > integralLimit) {
                integralSum = integralLimit * Math.signum(integralSum);
            }
        } else if (!activateIntegral) {
            integralSum = 0;
        }


        outputPositionalValue = (error * outtakeConstants.Kp) + (derivative * outtakeConstants.Kd) + (integralSum * outtakeConstants.Ki);

        if (Math.abs(outputPositionalValue) > outputLimit) {
            outputPositionalValue = outputLimit * Math.signum(outputPositionalValue);
        }


        lastError = error;
        lastTime = currentTime;
        errorChange = error - lastError;
        timeChange = deltaTime;

        return outputPositionalValue;
    }




    }









