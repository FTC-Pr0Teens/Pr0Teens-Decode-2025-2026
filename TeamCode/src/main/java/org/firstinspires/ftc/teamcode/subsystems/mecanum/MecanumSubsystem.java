package org.firstinspires.ftc.teamcode.subsystems.mecanum;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Specifications;

/**
 * MecanumSubsystem controls a mecanum wheel drivetrain.
 * Supports field-oriented and robot-oriented drive, velocity control, and layered velocity adjustments.
 */
public class MecanumSubsystem extends Specifications {
    // Motor controllers
    private DcMotorEx leftBack;      // Rear left motor
    private DcMotorEx rightBack;     // Rear right motor
    private DcMotorEx leftForward;   // Front left motor
    private DcMotorEx rightForward;  // Front right motor

    // Power scaling factor (from constants)
    private final double SCALE = MecanumConstants.SCALE;

    // Raw power outputs for each wheel
    public double rf = 0; // right front
    public double rb = 0; // right back
    public double lb = 0; // left back
    public double lf = 0; // left front

    // Velocity control variables (radians/sec)
    public double lfvel = 0;
    public double lbvel = 0;
    public double rfvel = 0;
    public double rbvel = 0;

    // Previous velocity values (for smoothing or feedback)
    public double previousLfvel = 0;
    public double previousLbvel = 0;
    public double previousRfvel = 0;
    public double previousRbvel = 0;

    // Velocity adjustment layers (for async or advanced control)
    public double lfvelMain = 0;            // Primary movement control
    public double lbvelMain = 0;
    public double rfvelMain = 0;
    public double rbvelMain = 0;

    public double lfvelAdjustment1 = 0;     // First adjustment layer
    public double lbvelAdjustment1 = 0;
    public double rfvelAdjustment1 = 0;
    public double rbvelAdjustment1 = 0;

    public double lfvelAdjustment2 = 0;     // Second adjustment layer
    public double lbvelAdjustment2 = 0;
    public double rfvelAdjustment2 = 0;
    public double rbvelAdjustment2 = 0;

    // Power values for each wheel (used in field-oriented drive)
    public double frontRightPow;
    public double frontLeftPow;
    public double backRightPow;
    public double backLeftPow;

    /**
     * Field-oriented drive: moves the robot relative to the field.
     * @param x     Side-to-side input (-1 to 1)
     * @param y     Forward-back input (-1 to 1)
     * @param z     Rotation input (-1 to 1)
     * @param theta Robot heading (radians)
     */
    public void fieldOrientedMove(double x, double y, double z, double theta) {
        // Convert field-relative input to robot-relative
        double newX = x * Math.cos(theta) - y * Math.sin(theta);
        double newY = x * Math.sin(theta) + y * Math.cos(theta);

        // Calculate wheel powers
        frontRightPow = -newY + newX - z;
        frontLeftPow = newY + newX + z;
        backRightPow = newY + newX - z;
        backLeftPow = -newY + newX + z;

        // Normalize powers to keep within [-1, 1]
        double largest = Math.max(
                Math.max(Math.abs(frontRightPow), Math.abs(frontLeftPow)),
                Math.max(Math.abs(backRightPow), Math.abs(backLeftPow)));
        if (largest > 1) {
            frontRightPow /= largest;
            frontLeftPow /= largest;
            backRightPow /= largest;
            backLeftPow /= largest;
        }

        // Apply scaling and set motor powers
        frontRightPow *= SCALE;
        frontLeftPow *= SCALE;
        backRightPow *= SCALE;
        backLeftPow *= SCALE;

        rightForward.setPower(frontRightPow);
        leftForward.setPower(frontLeftPow);
        rightBack.setPower(backRightPow);
        leftBack.setPower(backLeftPow);
    }

    /**
     * Field-oriented drive with exponential scaling for finer low-speed control.
     */
    public void fieldOrientedMoveExponential(double x, double y, double z, double theta){
        double newX = x * Math.cos(theta) - y * Math.sin(theta);
        double newY = x * Math.sin(theta) + y * Math.cos(theta);

        frontRightPow = - newY + newX - z;
        frontLeftPow = newY + newX + z;
        backRightPow = newY + newX -  z;
        backLeftPow = - newY + newX + z;

        double largest = Math.max(
                Math.max(Math.abs(frontRightPow), Math.abs(frontLeftPow)),
                Math.max(Math.abs(backRightPow), Math.abs(backLeftPow)));

        if (largest > 1) {
            frontRightPow /= largest;
            frontLeftPow /= largest;
            backRightPow /= largest;
            backLeftPow /= largest;
        }
        frontRightPow *= SCALE;
        frontLeftPow *= SCALE;
        backRightPow *= SCALE;
        backLeftPow *= SCALE;

        rightForward.setPower(normalizedFunction(frontRightPow));
        leftForward.setPower(normalizedFunction(frontLeftPow));
        rightBack.setPower(normalizedFunction(backRightPow));
        leftBack.setPower(normalizedFunction(backLeftPow));
    }

    /**
     * Exponential scaling function for joystick input.
     * @param t Input value (-1 to 1)
     * @return Scaled output
     */
    public static double normalizedFunction(double t) {
        double x = 128 * Math.abs(t);
        double numerator = 1.2 * Math.pow(1.02, x) - 1.2 + 0.2 * x;
        double denominator = 1.2 * Math.pow(1.02, 128) - 1.2 + 0.2 * 128;
        double result;
        if (t < 0) {
            result = numerator * -1 / denominator;
        } else {
            result = numerator / denominator;
        }
        return result;
    }

    /**
     * Processes velocity control with encoder feedback.
     * Combines main and adjustment velocities, limits to max, and sets motor velocities.
     */
    public void motorProcess(){
        // Combine main and adjustment velocities
        lfvel = lfvelMain + lfvelAdjustment1;
        lbvel = lbvelMain + lbvelAdjustment1;
        rfvel = rfvelMain + rfvelAdjustment1;
        rbvel = rbvelMain + rbvelAdjustment1;

        // Limit velocities to maximum allowed
        double max = Math.max(Math.abs(lfvel), Math.max(Math.abs(lbvel), Math.max(Math.abs(rfvel), Math.abs(rbvel))));
        if (max > MecanumConstants.MAX_ANGULAR_VEL) {
            double scalar = MecanumConstants.MAX_ANGULAR_VEL / max;
            lfvel *= scalar;
            lbvel *= scalar;
            rfvel *= scalar;
            rbvel *= scalar;
        }

        // Set motor velocities (radian/s)
        rightForward.setVelocity(rfvel, AngleUnit.RADIANS);
        leftBack.setVelocity(lbvel, AngleUnit.RADIANS);
        rightBack.setVelocity(rbvel, AngleUnit.RADIANS);
        leftForward.setVelocity(lfvel, AngleUnit.RADIANS);
    }

    /**
     * Processes velocity control with no encoder feedback.
     * Normalizes velocity vectors and sets motor powers.
     */
    public void motorProcessNoEncoder(){
        // Normalize vectors if any velocity exceeds 1
        if (Math.abs(lfvelMain + lfvelAdjustment1 + lfvelAdjustment2) > 1 ||
            Math.abs(lbvelMain + lbvelAdjustment1 + lbvelAdjustment2) > 1 ||
            Math.abs(rfvelMain + rfvelAdjustment1 + rfvelAdjustment2) > 1 ||
            Math.abs(rbvelMain + rbvelAdjustment1 + rbvelAdjustment2) > 1) {
            lfvel = (lfvelMain + lfvelAdjustment1 + lfvelAdjustment2);
            lbvel = (lbvelMain + lbvelAdjustment1 + lbvelAdjustment2);
            rfvel = (rfvelMain + rfvelAdjustment1 + rfvelAdjustment2);
            rbvel = (rbvelMain + rbvelAdjustment1 + rbvelAdjustment2);
            double max = Math.max(Math.abs(lfvel), Math.max(Math.abs(lbvel), Math.max(Math.abs(rfvel), Math.abs(rbvel))));
            lfvel = lfvel / max;
            lbvel = lbvel / max;
            rfvel = rfvel / max;
            rbvel = rbvel / max;
        } else {
            lfvel = lfvelMain + lfvelAdjustment1 + lfvelAdjustment2;
            lbvel = lbvelMain + lbvelAdjustment1 + lbvelAdjustment2;
            rfvel = rfvelMain + rfvelAdjustment1 + rfvelAdjustment2;
            rbvel = rbvelMain + rbvelAdjustment1 + rbvelAdjustment2;
        }

        // Set motor powers
        rightForward.setPower(rfvel);
        leftBack.setPower(lbvel);
        rightBack.setPower(rbvel);
        leftForward.setPower(lfvel);
    }

    /**
     * Clamps a value to a maximum change.
     * @param maxChange Maximum allowed change
     * @param difference Value to clamp
     * @return Clamped value
     */
    private double clamp(double maxChange, double difference) {
        if (difference > maxChange) {
            return maxChange;
        } else if (difference < -maxChange) {
            return -maxChange;
        } else {
            return difference;
        }
    }

    /**
     * Constructor: initializes all motors and sets directions, behaviors, and modes.
     * @param hardwareMap HardwareMap from OpMode
     */
    public MecanumSubsystem(HardwareMap hardwareMap) {
        // Initialize motors using names from constants
        leftBack = hardwareMap.get(DcMotorEx.class, MecanumConstants.BKLF_MOTOR);
        rightBack = hardwareMap.get(DcMotorEx.class, MecanumConstants.BKRT_MOTOR);
        leftForward = hardwareMap.get(DcMotorEx.class, MecanumConstants.FTLF_MOTOR);
        rightForward = hardwareMap.get(DcMotorEx.class, MecanumConstants.FTRT_MOTOR);

        // Set motor directions (adjust as needed for your robot)
        leftForward.setDirection(DcMotorSimple.Direction.REVERSE);
        rightForward.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior to BRAKE for all motors
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to run without encoder by default
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Stop all motors
        leftBack.setPower(0);
        leftForward.setPower(0);
        rightBack.setPower(0);
        rightForward.setPower(0);
    }

    /**
     * Disables internal PID by setting all motors to RUN_WITHOUT_ENCODER.
     */
    public void turnOffInternalPID() {
        rightForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Robot-oriented drive (main driver control).
     * @param run Whether to run the drive
     * @param vertical Forward/backward input
     * @param horizontal Side-to-side input
     * @param rotational Rotation input
     */
    public void move(boolean run, double vertical, double horizontal, double rotational){
        if (run){
            rb = (vertical * Math.cos(Math.toRadians(45)) + horizontal * Math.sin(Math.toRadians(45)) + rotational * Math.sin(Math.toRadians(45))) * 1.41421356237;
            rf = (-horizontal * Math.cos(Math.toRadians(45)) + vertical * Math.sin(Math.toRadians(45)) + rotational * Math.sin(Math.toRadians(45))) * 1.41421356237;
            lf = (vertical * Math.cos(Math.toRadians(45)) + horizontal * Math.sin(Math.toRadians(45)) - rotational * Math.sin(Math.toRadians(45))) * 1.41421356237;
            lb = (-horizontal * Math.cos(Math.toRadians(45)) + vertical * Math.sin(Math.toRadians(45)) - rotational * Math.sin(Math.toRadians(45))) * 1.41421356237;

            rightForward.setPower(rf);
            leftBack.setPower(lb);
            rightBack.setPower(rb);
            leftForward.setPower(lf);
        }
    }

    /**
     * Robot-oriented drive using velocity control.
     */
    public void moveVelocity(boolean run, double vertical, double horizontal, double rotational){
        if (run){
            rb = (vertical * Math.cos(Math.toRadians(45)) + horizontal * Math.sin(Math.toRadians(45)) + rotational * Math.sin(Math.toRadians(45))) * 1.41421356237;
            rf = (-horizontal * Math.cos(Math.toRadians(45)) + vertical * Math.sin(Math.toRadians(45)) + rotational * Math.sin(Math.toRadians(45))) * 1.41421356237;
            lf = (vertical * Math.cos(Math.toRadians(45)) + horizontal * Math.sin(Math.toRadians(45)) - rotational * Math.sin(Math.toRadians(45))) * 1.41421356237;
            lb = (-horizontal * Math.cos(Math.toRadians(45)) + vertical * Math.sin(Math.toRadians(45)) - rotational * Math.sin(Math.toRadians(45))) * 1.41421356237;

            rightForward.setVelocity(rf);
            leftBack.setVelocity(lb);
            rightBack.setVelocity(rb);
            leftForward.setVelocity(lf);
        }
    }

    /**
     * Partial move for main velocity layer (used for async or advanced control).
     */
    public void partialMove(boolean run, double verticalVel, double horizontalVel, double rotationalVel){
        if (run){
            rbvelMain = (verticalVel * Math.cos(Math.toRadians(45)) + horizontalVel * Math.sin(Math.toRadians(45)) + rotationalVel * Math.sin(Math.toRadians(45))) * 1.41421356237;
            rfvelMain = (-horizontalVel * Math.cos(Math.toRadians(45)) + verticalVel * Math.sin(Math.toRadians(45)) + rotationalVel * Math.sin(Math.toRadians(45))) * 1.41421356237;
            lfvelMain = (verticalVel * Math.cos(Math.toRadians(45)) + horizontalVel * Math.sin(Math.toRadians(45)) - rotationalVel * Math.sin(Math.toRadians(45))) * 1.41421356237;
            lbvelMain = (-horizontalVel * Math.cos(Math.toRadians(45)) + verticalVel * Math.sin(Math.toRadians(45)) - rotationalVel * Math.sin(Math.toRadians(45))) * 1.41421356237;
        }
    }

    /**
     * Partial move for adjustment layer 1.
     */
    public void partialMoveAdjustment1(boolean run, double verticalVel, double horizontalVel, double rotationalVel){
        if (run){
            rbvelAdjustment1 = (verticalVel * Math.cos(Math.toRadians(45)) + horizontalVel * Math.sin(Math.toRadians(45)) + rotationalVel * Math.sin(Math.toRadians(45))) * 1.41421356237;
            rfvelAdjustment1 = (-horizontalVel * Math.cos(Math.toRadians(45)) + verticalVel * Math.sin(Math.toRadians(45)) + rotationalVel * Math.sin(Math.toRadians(45))) * 1.41421356237;
            lfvelAdjustment1 = (verticalVel * Math.cos(Math.toRadians(45)) + horizontalVel * Math.sin(Math.toRadians(45)) - rotationalVel * Math.sin(Math.toRadians(45))) * 1.41421356237;
            lbvelAdjustment1 = (-horizontalVel * Math.cos(Math.toRadians(45)) + verticalVel * Math.sin(Math.toRadians(45)) - rotationalVel * Math.sin(Math.toRadians(45))) * 1.41421356237;
        }
    }

    /**
     * Partial move for adjustment layer 2.
     */
    public void partialMoveAdjustment2(boolean run, double verticalVel, double horizontalVel, double rotationalVel){
        if (run){
            rbvelAdjustment2 = (verticalVel * Math.cos(Math.toRadians(45)) + horizontalVel * Math.sin(Math.toRadians(45)) + rotationalVel * Math.sin(Math.toRadians(45))) * 1.41421356237;
            rfvelAdjustment2 = (-horizontalVel * Math.cos(Math.toRadians(45)) + verticalVel * Math.sin(Math.toRadians(45)) + rotationalVel * Math.sin(Math.toRadians(45))) * 1.41421356237;
            lfvelAdjustment2 = (verticalVel * Math.cos(Math.toRadians(45)) + horizontalVel * Math.sin(Math.toRadians(45)) - rotationalVel * Math.sin(Math.toRadians(45))) * 1.41421356237;
            lbvelAdjustment2 = (-horizontalVel * Math.cos(Math.toRadians(45)) + verticalVel * Math.sin(Math.toRadians(45)) - rotationalVel * Math.sin(Math.toRadians(45))) * 1.41421356237;
        }
    }

    /**
     * TeleOp tank drive move.
     */
    public void move(boolean run, double leftVertical, double leftHorizontal, double rightVertical, double rightHorizontal){
        if (run){
            rf = rightHorizontal + rightVertical;
            lb = leftVertical + leftHorizontal;
            rb = rightVertical - rightHorizontal;
            lf = leftVertical - leftHorizontal;

            rightForward.setPower(rf);
            leftBack.setPower(lb);
            rightBack.setPower(rb);
            leftForward.setPower(lf);
        }
    }

    /**
     * Move at a specific angle while facing forward.
     */
    public void moveAngle(boolean run, double power, double degree){
        if (run){
            double y1 = power * Math.sin(Math.toRadians(degree)) * Math.cos(Math.toRadians(45)) - power * Math.cos(Math.toRadians(degree)) * Math.sin(Math.toRadians(45));
            double x1 = power * Math.cos(Math.toRadians(degree)) * Math.cos(Math.toRadians(45)) + power * Math.sin(Math.toRadians(degree)) * Math.sin(Math.toRadians(45));
            double y2 = power * Math.sin(Math.toRadians(degree)) * Math.cos(Math.toRadians(45)) - power * Math.cos(Math.toRadians(degree)) * Math.sin(Math.toRadians(45));
            double x2 = power * Math.cos(Math.toRadians(degree)) * Math.cos(Math.toRadians(45)) + power * Math.sin(Math.toRadians(degree)) * Math.sin(Math.toRadians(45));
            rightForward.setPower(x1);
            leftBack.setPower(x2);
            rightBack.setPower(y1);
            leftForward.setPower(y2);
        }
    }

    /**
     * Move to a position (blocking, not recommended for OpMode).
     */
    public void moveToPosition(boolean run, double power, double degree, int position){
        if (run){
            double y1 = power * Math.sin(Math.toRadians(degree)) * Math.cos(Math.toRadians(45)) - power * Math.cos(Math.toRadians(degree)) * Math.sin(Math.toRadians(45));
            double x1 = power * Math.cos(Math.toRadians(degree)) * Math.cos(Math.toRadians(45)) + power * Math.sin(Math.toRadians(degree)) * Math.sin(Math.toRadians(45));
            double y2 = power * Math.sin(Math.toRadians(degree)) * Math.cos(Math.toRadians(45)) - power * Math.cos(Math.toRadians(degree)) * Math.sin(Math.toRadians(45));
            double x2 = power * Math.cos(Math.toRadians(degree)) * Math.cos(Math.toRadians(45)) + power * Math.sin(Math.toRadians(degree)) * Math.sin(Math.toRadians(45));
            while (rightForward.getCurrentPosition() < position){
                rightForward.setPower(x1);
                leftBack.setPower(x2);
                rightBack.setPower(y1);
                leftForward.setPower(y2);
            }
            rightForward.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            leftForward.setPower(0);
        }
    }

    /**
     * Stops all motors.
     */
    public void stop(boolean run){
        if (run){
            rightForward.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            leftForward.setPower(0);
        }
    }

    /**
     * Resets all motor encoders.
     */
    public void reset(){
        rightForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // --- Telemetry and accessors ---

    public double rightForwardPow() { return rightForward.getPower(); }
    public double rightBackPow() { return rightBack.getPower(); }
    public double leftForwardPow() { return leftForward.getPower(); }
    public double leftBackPow() { return leftBack.getPower(); }

    public DcMotorEx getLeftBack() { return leftBack; }
    public DcMotorEx getRightBack() { return rightBack; }
    public DcMotorEx getLeftForward() { return leftForward; }
    public DcMotorEx getRightForward() { return rightForward; }

    public double getLeftBackVelocity() { return leftBack.getVelocity(AngleUnit.RADIANS); }
    public double getRightBackVelocity() { return rightBack.getVelocity(AngleUnit.RADIANS); }
    public double getLeftForwardVelocity() { return leftForward.getVelocity(AngleUnit.RADIANS); }
    public double getRightForwardVelocity() { return rightForward.getVelocity(AngleUnit.RADIANS); }

    public double rightForwardPos() { return rightForward.getCurrentPosition(); }
    public double rightBackPos() { return rightBack.getCurrentPosition(); }
    public double leftForwardPos() { return leftForward.getCurrentPosition(); }
    public double leftBackPos() { return leftBack.getCurrentPosition(); }
}