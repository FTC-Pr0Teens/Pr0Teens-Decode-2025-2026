package org.firstinspires.ftc.teamcode.subsystems.lift;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.util.pidcore.PIDCore;

class LiftTest {
    private final Hardware hw;

    // Shooter components (using existing hardware)
    private DcMotorEx shooterLeft;   // lout
    private DcMotorEx shooterRight;  // rout
    private CRServo feederServo;     // turret
    private Servo shooterAngleServo;

    // Shooter PID controller
    private PIDCore shooterPID;

    // Shooter constants
    private static final double SHOOTER_KP = 0.001;
    private static final double SHOOTER_KI = 0.0001;
    private static final double SHOOTER_KD = 0.0;
    private static final double MAX_SHOOTER_RPM = 5400; // Adjust based on your motor
    private static final double SHOOTER_TOLERANCE = 50; // RPM tolerance
    private static final double SHOOTER_ANGLE_MIN = 0.0; // Minimum servo position
    private static final double SHOOTER_ANGLE_MAX = 1.0; // Maximum servo position

    public LiftSubsystem(Hardware hw) {
        this.hw = hw;

        // Use existing hardware for shooter functionality
        this.shooterLeft = hw.lout;    // Left shooter (lout)
        this.shooterRight = hw.rout;   // Right shooter (rout)
        this.feederServo = hw.turret;  // Feeder (turret)


        // Initialize PID controller for shooter
        shooterPID = new PIDCore(SHOOTER_KP, SHOOTER_KD, SHOOTER_KI);
    }

    LiftTest(Hardware hw) {
        this.hw = hw;
    }

    public void intake() {
        hw.intake.setPower(1.0);
    }

    public void stopintake() {
        hw.intake.setPower(0.0);
    }


    public void outtake() {
        hw.lout.setPower(-0.8);
        hw.rout.setPower(0.8);
    }

    public void outtakestop() {
        hw.lout.setPower(0.0);
        hw.rout.setPower(0.0);
    }


    public void turret() {
        hw.turret.setPower(1.0);
    }

    public void turretstop() {
        hw.turret.setPower(0.0);
    }

    public void push() {
        hw.push.setDirection(Servo.Direction.REVERSE);
        hw.push.setPosition(0.6);
    }

    public void pull() {
        hw.push.setPosition(0.27);
    }


    public void setShooterVelocity(double velocityPercent) {
        if (shooterLeft == null || shooterRight == null) return;

        double targetRPM = velocityPercent * MAX_SHOOTER_RPM;


        shooterLeft.setVelocity(targetRPM, AngleUnit.DEGREES);
        shooterRight.setVelocity(targetRPM, AngleUnit.DEGREES);


    }

    public void stopShooter() {
        if (shooterLeft == null || shooterRight == null) return;

        shooterLeft.setPower(0.0);
        shooterRight.setPower(0.0);
    }

    public void feedShooter() {
        if (feederServo == null) return;
        feederServo.setPower(FEEDER_POWER_SCALE);
    }

    public void stopFeeder() {
        if (feederServo == null) return;
        feederServo.setPower(0.0);
    }

    public void setShooterAngle(double angleDegrees) {
        if (shooterAngleServo == null) return;

        // Convert angle to servo position (you'll need to calibrate this)
        double servoPosition = mapAngleToServoPosition(angleDegrees);
        servoPosition = Math.max(SHOOTER_ANGLE_MIN, Math.min(SHOOTER_ANGLE_MAX, servoPosition));
        shooterAngleServo.setPosition(servoPosition);
    }

    private double mapAngleToServoPosition(double angleDegrees) {
        // Map angle in degrees to servo position (0.0 to 1.0)
        // This is a linear mapping - adjust based on your servo/mechanism
        double minAngleDeg = 0.0;   // Minimum angle your mechanism can achieve
        double maxAngleDeg = 60.0;  // Maximum angle your mechanism can achieve

        return (angleDegrees - minAngleDeg) / (maxAngleDeg - minAngleDeg);
    }

    public double getShooterVelocity() {
        if (shooterLeft == null) return 0.0;

        // Return average velocity of both motors in RPM
        double velocity1 = shooterLeft.getVelocity(AngleUnit.DEGREES) / 6.0; // Convert to RPM
        if (shooterRight != null) {
            double velocity2 = shooterRight.getVelocity(AngleUnit.DEGREES) / 6.0;
            return (velocity1 + velocity2) / 2.0;
        }
        return velocity1;
    }

    public boolean isShooterAtSpeed(double targetVelocityPercent) {
        if (shooterLeft == null) return false;

        double targetRPM = targetVelocityPercent * MAX_SHOOTER_RPM;
        double currentRPM = getShooterVelocity();

        return Math.abs(targetRPM - currentRPM) < SHOOTER_TOLERANCE;
    }

    public void update() {

    }

    // Constants for feeder and shooter
    private static final double FEEDER_POWER_SCALE = 0.8;
}