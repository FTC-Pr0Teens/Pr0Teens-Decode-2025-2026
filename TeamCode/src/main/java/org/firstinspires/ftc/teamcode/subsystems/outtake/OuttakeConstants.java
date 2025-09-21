package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.qualcomm.robotcore.util.ElapsedTime;

public class OuttakeConstants {
    public static double Kd = 0.01;
    public static double Kp = 0.03;
    public static double Ki = 0.00 ;

    // Proportional, Derivative, Integral gains for velocity control
    public double KpVel;
    public double KdVel;
    public double KiVel;

    // Feedforward constants for acceleration, velocity, and gravity compensation
    public double Ka;
    public double Kv;
    public double Kg;
    public double Kf; // Feedforward gain

    // Timer to track elapsed time between PID calculations
    public ElapsedTime timer;
    public ElapsedTime integralTimer;

    // PID State Variables
    public double error = 0;
    public double lastError = 0;
    public double lastVelError = 0;
    public double integralSum = 0;
    public double derivative = 0;
    public double velocityDerivative = 0;

    // Output tracking
    public double outputPositionalValue = 0;
    public double outputVelocityValue = 0;
    public double feedForward = 0;

    // Control flags
    public boolean activateIntegral = false;

    // Safety limits
    public double integralLimit = 1000.0; // Prevent integral windup
    public double outputLimit = 1.0;      // Limit output magnitude
    public double deadband = 0.0;         // Ignore small errors

    // Time tracking
    public double lastTime = 0;
    public double timeChange = 0;
    public double errorChange = 0;
}
