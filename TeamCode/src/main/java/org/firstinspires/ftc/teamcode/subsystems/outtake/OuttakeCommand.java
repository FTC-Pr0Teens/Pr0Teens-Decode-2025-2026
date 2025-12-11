package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

public class OuttakeCommand {

    private Hardware hw;

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
    private static final double PUSHER_UP = 0.2;
    private static final double PUSHER_DOWN = 0.0;
    private static final double PUSHER_UP1 = 0.2;
    private static final double PUSHER_DOWN1 = 0;
    private static final long PUSHER_TIME = 400;
    private static final long SORTER_TIME = 400;

    private int sorterpos = 0;
    private TransferState state = TransferState.FIRST;

    private enum TransferState {
        PUSH_UP,
        PUSH_DOWN,
        SORT,
        FIRST
    }

    public OuttakeCommand(Hardware hw) {
        this.hw = hw;
        this.shooter = hw.shooter;
        this.shooter2 = hw.shooter2;
        this.targetRPM = DEFAULT_RPM;
        this.targetRPM1 = DEFAULT_RPM1;
        hw.shooter.setVelocityPIDFCoefficients(75, 10.0, 10.0, 0.0);
        hw.shooter2.setVelocityPIDFCoefficients(75, 10.0, 10.0, 0.0);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        timer = new ElapsedTime();
        hw.pusher1.setDirection(Servo.Direction.REVERSE);

        isPusherUp = false;
        firstRun = true;
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

    public void setMaxRPM(double maxRPM) {
        targetRPM = maxRPM;
    }

    public double getShooterRPM(double distance) {

        double[] dist = {60, 80, 90, 110, 130, 150};
        double[] rpm  = {2400, 2500, 2600, 2800, 3000, 5300};

        // If outside the range:
        if (distance <= dist[0]) return rpm[0];
        if (distance >= dist[dist.length - 1]) return rpm[rpm.length - 1];

        // Otherwise interpolate between points
        for (int i = 0; i < dist.length - 1; i++) {
            if (distance >= dist[i] && distance <= dist[i+1]) {
                return lerp(distance, dist[i], dist[i+1], rpm[i], rpm[i+1]);
            }
        }

        return rpm[0];
    }


    public boolean transfer() {
        switch (state) {
            case FIRST:
                pusherUp();
                pusherTimer.reset();
                state = TransferState.PUSH_UP;
                return true;

            case PUSH_UP:
                if (pusherTimer.milliseconds() >= PUSHER_TIME) {
                    pusherDown();
                    sorterTimer.reset();
                    state = TransferState.PUSH_DOWN;
                }
                return true;

            case PUSH_DOWN:
                if (sorterTimer.milliseconds() >= SORTER_TIME) {
                    state = TransferState.SORT;
                }
                return true;

            case SORT:
                sorterpos = (sorterpos + 1) % 3;
                if (sorterpos == 0) {
                    hw.sorter.setPosition(SORTER_FIRST_POS);
                }
                if (sorterpos == 1) {
                    hw.sorter.setPosition(SORTER_SECOND_POS);
                }
                if (sorterpos == 2) {
                    hw.sorter.setPosition(SORTER_THIRD_POS);
                }

                state = TransferState.FIRST;
                return false;

            default:
                return false;
        }
    }

    public void sorter(boolean check){
        if (check && sorterTimer.milliseconds() > SORTER_TIME && !isPusherUp) {
            sorterTimer.reset();
            sorterpos = (sorterpos + 1) % 3;
            if (sorterpos == 0) {
                hw.sorter.setPosition(SORTER_FIRST_POS);//60 degrees
            } else if (sorterpos == 1) {
                hw.sorter.setPosition(SORTER_SECOND_POS);//60 degrees
            } else if (sorterpos == 2) {
                hw.sorter.setPosition(SORTER_THIRD_POS);//60 degrees
            }
        }
    }


    public void pusherUp() {
        hw.pusher.setPosition(PUSHER_UP);
        hw.pusher1.setPosition(PUSHER_UP1);
    }

    public void pusherDown() {
        hw.pusher.setPosition(PUSHER_DOWN);
        hw.pusher1.setPosition(PUSHER_DOWN1);
    }

    public double lerp(double x, double x0, double x1, double y0, double y1) {
        return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
    }

}







