package org.firstinspires.ftc.teamcode.util.pidcore;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.VelAccelPair;

/**
 * Enhanced PIDCore class with bug fixes and improvements for FTC robotics applications
 * Supports both positional and velocity PID control with advanced features
 */
public class PIDCore {
    // Proportional, Derivative, Integral gains for position control
    private double Kp;
    private double Kd;
    private double Ki;

    // Proportional, Derivative, Integral gains for velocity control
    private double KpVel;
    private double KdVel;
    private double KiVel;

    // Feedforward constants for acceleration, velocity, and gravity compensation
    private double Ka;
    private double Kv;
    private double Kg;
    private double Kf; // Feedforward gain

    // Timer to track elapsed time between PID calculations
    private ElapsedTime timer;
    private ElapsedTime integralTimer;

    // PID State Variables
    private double error = 0;
    private double lastError = 0;
    private double lastVelError = 0;
    private double integralSum = 0;
    private double derivative = 0;
    private double velocityDerivative = 0;

    // Output tracking
    private double outputPositionalValue = 0;
    private double outputVelocityValue = 0;
    private double feedForward = 0;

    // Control flags
    private boolean activateIntegral = false;

    // Safety limits
    private double integralLimit = 1000.0; // Prevent integral windup
    private double outputLimit = 1.0;      // Limit output magnitude
    private double deadband = 0.0;         // Ignore small errors

    // Time tracking
    private double lastTime = 0;
    private double timeChange = 0;
    private double errorChange = 0;

    /**
     * Constructor initializing basic PID gains (Kp, Kd, Ki).
     */
    public PIDCore(double kp, double kd, double ki) {
        this.Kp = kp;
        this.Kd = kd;
        this.Ki = ki;
        timer = new ElapsedTime();
        integralTimer = new ElapsedTime();
        reset();
    }

    /**
     * Constructor initializing PID gains and feedforward constants for acceleration, velocity, and gravity.
     */
    public PIDCore(double kp, double kd, double ki, double ka, double kv, double kg) {
        this.Kp = kp;
        this.Kd = kd;
        this.Ki = ki;
        this.Ka = ka;
        this.Kv = kv;
        this.Kg = kg;
        timer = new ElapsedTime();
        integralTimer = new ElapsedTime();
        reset();
    }

    /**
     * Constructor initializing PID gains and feedforward gain.
     */
    public PIDCore(double kp, double kd, double ki, double kf) {
        this.Kp = kp;
        this.Kd = kd;
        this.Ki = ki;
        this.Kf = kf;
        timer = new ElapsedTime();
        integralTimer = new ElapsedTime();
        reset();
    }

    /**
     * Constructor initializing position PID gains, velocity PID gains, and feedforward gain.
     */
    public PIDCore(double kp, double kd, double ki, double kpVel, double kdVel, double kiVel, double kf) {
        this.Kp = kp;
        this.Kd = kd;
        this.Ki = ki;
        this.KpVel = kpVel;
        this.KdVel = kdVel;
        this.KiVel = kiVel;
        this.Kf = kf;
        timer = new ElapsedTime();
        integralTimer = new ElapsedTime();
        reset();
    }

    /**
     * FIXED: Reset method to properly initialize PID state
     */
    public void reset() {
        timer.reset();
        integralTimer.reset();
        lastTime = 0;
        lastError = 0;
        lastVelError = 0;
        integralSum = 0;
        derivative = 0;
        velocityDerivative = 0;
        error = 0;
        outputPositionalValue = 0;
        outputVelocityValue = 0;
        feedForward = 0;
    }


    public double outputPositional(double setPoint, double feedback) {
        error = setPoint - feedback;

        // Apply deadband
        if (Math.abs(error) < deadband) {
            error = 0;
        }

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


        outputPositionalValue = (error * Kp) + (derivative * Kd) + (integralSum * Ki);

        if (Math.abs(outputPositionalValue) > outputLimit) {
            outputPositionalValue = outputLimit * Math.signum(outputPositionalValue);
        }


        lastError = error;
        lastTime = currentTime;
        errorChange = error - lastError;
        timeChange = deltaTime;

        return outputPositionalValue;
    }


    public double outputVelocity(double setVelocity, double feedback) {
        error = setVelocity - feedback;

        double currentTime = timer.seconds();
        double deltaTime = currentTime - lastTime;
        if (deltaTime <= 0) {
            deltaTime = 0.001;
        }


        integralSum += error * deltaTime;
        if (Math.abs(integralSum) > integralLimit) {
            integralSum = integralLimit * Math.signum(integralSum);
        }

        velocityDerivative = (error - lastVelError) / deltaTime;

        outputVelocityValue = (error * KpVel) + (velocityDerivative * KdVel) + (integralSum * KiVel);

        if (Kf != 0) {
            outputVelocityValue += Kf * feedback;
        }

        lastVelError = error;
        lastTime = currentTime;

        return outputVelocityValue;
    }


    public double outputVelocity(double setVelocity, double feedback, double power) {
        return power + outputVelocity(setVelocity, feedback);
    }

    public double outputFeedForward(double setPoint, double feedback, VelAccelPair velAccelPair) {

        double pidOutput = outputPositional(setPoint, feedback);


        feedForward = 0;
        if (Ka != 0) feedForward += Ka * velAccelPair.getAcceleration();
        if (Kv != 0) feedForward += Kv * velAccelPair.getVelocity();
        if (Kg != 0) feedForward += Kg;

        return pidOutput + feedForward;
    }

    public double cascadeOutput(double setPoint, double feedback, double setVelocity, double feedbackVelocity) {
        outputPositionalValue = outputPositional(setPoint, feedback);
        outputVelocityValue = outputVelocity(setVelocity, feedbackVelocity);
        return outputPositionalValue + outputVelocityValue;
    }

    // CONFIGURATION METHODS
    public void setConstant(double kp, double kd, double ki) {
        this.Kp = kp;
        this.Kd = kd;
        this.Ki = ki;
    }

    public void setConstant(double kp, double kd, double ki, double kf) {
        this.Kp = kp;
        this.Kd = kd;
        this.Ki = ki;
        this.Kf = kf;
    }

    public void setIntegralLimit(double limit) {
        this.integralLimit = Math.abs(limit);
    }

    public void setOutputLimit(double limit) {
        this.outputLimit = Math.abs(limit);
    }

    public void setDeadband(double deadband) {
        this.deadband = Math.abs(deadband);
    }

    public void activateIntegral() {
        activateIntegral = true;
    }

    public void deactivateIntegral() {
        activateIntegral = false;
        integralSum = 0; // Clear integral when deactivated
    }

    public void integralReset() {
        integralSum = 0;
    }

    // GETTER METHODS
    public double getKp() { return Kp; }
    public double getKd() { return Kd; }
    public double getKi() { return Ki; }
    public double getKg() { return Kg; }
    public double getError() { return error; }
    public double getLastError() { return lastError; }
    public double getDerivative() { return derivative; }
    public double getIntegralSum() { return integralSum; }
    public double getOutputPositionalValue() { return outputPositionalValue; }
    public double getOutputVelocityValue() { return outputVelocityValue; }
    public double getFeedForwardOutput() { return feedForward; }
    public boolean getActiveIntegral() { return activateIntegral; }
    public double getErrorChange() { return errorChange; }
    public double getChangeInTime() { return timeChange; }
    public double getVelocityDerivative() { return velocityDerivative; }
    public double getDerivativeTimer() { return timer.milliseconds(); }

    // LEGACY METHODS (for backward compatibility)
    public double outputPositional2(double setPoint, double feedback) {
        return outputPositional(setPoint, feedback);
    }

    public double outputPositionalFeedForward(double setPoint, double feedback) {
        double pidOutput = outputPositional(setPoint, feedback);
        return pidOutput + Kf;
    }

    public double outputPositionalCapped(double setPoint, double feedback, double integralCap) {
        double oldLimit = integralLimit;
        setIntegralLimit(integralCap);
        double result = outputPositional(setPoint, feedback);
        setIntegralLimit(oldLimit);
        return result;
    }

    public double outputPID(double error) {
        this.error = error;
        double currentTime = timer.seconds();
        double deltaTime = currentTime - lastTime;
        if (deltaTime <= 0) deltaTime = 0.001;

        derivative = (error - lastError) / deltaTime;

        lastError = error;
        lastTime = currentTime;

        return (error * Kp) + (derivative * Kd);
    }
}