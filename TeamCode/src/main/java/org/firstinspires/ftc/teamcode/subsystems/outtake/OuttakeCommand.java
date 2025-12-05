package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

public class OuttakeCommand {

    private Hardware hw;
    private OuttakeSubsystem outtakeSubsystem;

    private DcMotorEx shooter;
    private DcMotorEx shooter2;
    private double targetRPM;
    private double targetRPM1;

    double DEFAULT_RPM = 3000;
    double DEFAULT_RPM1 = 4500;

    double PPR_of_6000_motor = 28.0;

    double seconds_In_A_Minute = 60.0;
    public static double Kd = 0.0;
    public static double Kp = 0.8;
    public static double Ki = 0.00;
    private double error = 0;
    private double lastError = 0;
    private double integralSum = 0;
    // Output tracking
    private double outputPositionalValue = 0;
    private double derivative = 0;
    // Control flags
    private boolean activateIntegral = false;
    // Safety limits
    private double integralLimit = 1000.0; // Prevent integral windup
    private double outputLimit = 1.0;      // Limit output magnitude
    // Ignore small errors

    // Time tracking
    private double lastTime = 0;
    private double timeChange = 0;
    private double errorChange = 0;
    private ElapsedTime timer;

    private final ElapsedTime pusherTimer = new ElapsedTime();
    private final ElapsedTime sorterTimer = new ElapsedTime();
    private static final double SORTER_FIRST_POS = 0.0;
    private static final double SORTER_SECOND_POS = 0.45;
    private static final double SORTER_THIRD_POS = 0.90;

    private boolean isPusherUp = false;
    private boolean firstRun = true; // ADD THIS
    private static final double PUSHER_UP = 0.39;
    private static final double PUSHER_DOWN = 0.0;
    private static final double PUSHER_UP1 = 0.19;
    private static final double PUSHER_DOWN1 = 0;
    private static final long PUSHER_TIME = 500;

    private int sorterpos = 0;

    private boolean pushersequence = false;

    public OuttakeCommand(Hardware hw) {
        this.hw = hw;
        this.shooter = hw.shooter;
        this.shooter2 = hw.shooter2;
        this.targetRPM = DEFAULT_RPM;
        this.targetRPM1 = DEFAULT_RPM1;
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        timer = new ElapsedTime();
        hw.pusher1.setDirection(Servo.Direction.REVERSE);

        isPusherUp = false;
        firstRun = true;
        sorterTimer.reset();
        pusherTimer.reset();
    }

    //returns whether or not we have reached the correctRPM
    public boolean isRPMReached() {
        double currentRPM = hw.shooter.getVelocity() * seconds_In_A_Minute / PPR_of_6000_motor;
        return Math.abs(targetRPM - currentRPM) < 200;
    }

    public boolean isRPMReachedFar() {
        double currentRPM = hw.shooter.getVelocity() * seconds_In_A_Minute / PPR_of_6000_motor;
        return Math.abs(targetRPM1 - currentRPM) < 200;
    }


    public boolean spinup() {
        double targetTPS = targetRPM * PPR_of_6000_motor / seconds_In_A_Minute;
        hw.shooter.setVelocity(targetTPS);
        hw.shooter2.setVelocity(targetTPS);

        return isRPMReached();
    }

    public boolean spinupfar() {
        double targetTPS = targetRPM1 * PPR_of_6000_motor / seconds_In_A_Minute;
        hw.shooter.setVelocity(targetTPS);
        return isRPMReachedFar();
    }


    public void stopShooter() {
        hw.shooter.setVelocity(0);
        hw.shooter2.setVelocity(0);
    }

    public void setMaxRPM(int maxRPM) {
        targetRPM = maxRPM;
    }

    public double dualShooterPID(double targetRPM, double currentRPM) {
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
    public boolean transfer() {
        // first run
        if (!pushersequence){
            pushersequence = true;
        }
        if (!isPusherUp && pusherTimer.milliseconds() > 800) {
            pusherUp();
            pusherTimer.reset();
            isPusherUp = true;
            return true;
        }

        if (isPusherUp && pusherTimer.milliseconds() >= PUSHER_TIME) {
            pusherDown();
            isPusherUp = false;
            sorterTimer.reset();
            return true;
        }

//        if (!isPusherUp && sorterTimer.milliseconds() > 800) {
//            if (sorterpos == 0) {
//                hw.sorter.setPosition(SORTER_FIRST_POS);
//            } else if (sorterpos == 1) {
//                hw.sorter.setPosition(SORTER_SECOND_POS);
//            } else if (sorterpos == 2) {
//                hw.sorter.setPosition(SORTER_THIRD_POS);
//            }
//            sorterpos = (sorterpos + 1) % 3;
//            return false;
//        }

        return true;
    }

    public void pusherUp() {
        hw.pusher.setPosition(PUSHER_UP);
        hw.pusher1.setPosition(PUSHER_UP1);
    }

    public void pusherDown() {
        hw.pusher.setPosition(PUSHER_DOWN);
        hw.pusher1.setPosition(PUSHER_DOWN1);
    }
}







