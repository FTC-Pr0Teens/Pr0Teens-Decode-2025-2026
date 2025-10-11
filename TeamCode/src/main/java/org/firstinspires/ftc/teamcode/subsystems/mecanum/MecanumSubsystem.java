package org.firstinspires.ftc.teamcode.subsystems.mecanum;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import static org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumConstants.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.util.pidcore.PIDCore;

import java.util.Arrays;
import java.util.Collections;

class MecanumSubsystem {
    //rf: right front/forward
    //rb: right back
    //lb: left back
    //lf: left front/forward
    //vel: velocity
    //Main: driver controlled or main program
    //Adjustment: async process controlled

    // --- Dependencies ---
    private final Hardware hw;

    // --- PID Controllers ---
    private final PIDCore globalXController;
    private final PIDCore globalYController;
    private final PIDCore globalThetaController;

    // --- Internal State: Raw Motor Outputs (Power or Velocity) ---
    public double rightFrontMotorOutput = 0;
    public double rightBackMotorOutput = 0;
    public double leftBackMotorOutput = 0;
    public double leftFrontMotorOutput = 0;

    // --- Target Wheel Velocities (radians/s) ---
    private double lfvel = 0;
    private double lbvel = 0;
    private double rfvel = 0;
    private double rbvel = 0;

    // --- Internal State: Layered Velocity Adjustments (radians/s) ---
    // Layer 0: Main driver/autonomous control input
    private double lfVelMain = 0;
    private double lbVelMain = 0;
    private double rfVelMain = 0;
    private double rbVelMain = 0;

    // Layer 1: First adjustment layer (e.g., heading correction, sensor-based adjustment)
    private double lfVelAdjustment1 = 0;
    private double lbVelAdjustment1 = 0;
    private double rfVelAdjustment1 = 0;
    private double rbVelAdjustment1 = 0;

    public MecanumSubsystem(Hardware hw) {
        this.hw = hw;

        // initialize PID controllers
        globalXController = new PIDCore(kpx, kdx, kix);
        globalYController = new PIDCore(kpy, kdy, kiy);
        globalThetaController = new PIDCore(kptheta, kdtheta, kitheta);

        hw.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.lb.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.rf.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.rb.setDirection(DcMotorSimple.Direction.REVERSE);


        // set motor behaviour
        hw.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set motor modes
        hw.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // stop all motors
        hw.lb.setPower(0);
        hw.lf.setPower(0);
        hw.rb.setPower(0);
        hw.rf.setPower(0);
    }

    // provides more control at lower speeds
    public void fieldOrientedMoveExponential(double x, double y, double z, double theta){
        double newX = x * Math.cos(theta) - y * Math.sin(theta);
        double newY = x * Math.sin(theta) + y * Math.cos(theta);

        rightFrontMotorOutput = - newY + newX - z;
        leftFrontMotorOutput = newY + newX + z;
        rightBackMotorOutput = newY + newX -  z;
        leftBackMotorOutput = - newY + newX + z;

        double largest = Math.max(
                Math.max(Math.abs(rightFrontMotorOutput), Math.abs(leftFrontMotorOutput)),
                Math.max(Math.abs(rightBackMotorOutput), Math.abs(leftBackMotorOutput)));

        if (largest > 1) {
            rightFrontMotorOutput /= largest;
            leftFrontMotorOutput /= largest;
            rightBackMotorOutput /= largest;
            leftBackMotorOutput /= largest;
        }
        rightFrontMotorOutput *= POWER_SCALE_FACTOR;
        leftFrontMotorOutput *= POWER_SCALE_FACTOR;
        rightBackMotorOutput *= POWER_SCALE_FACTOR;
        leftBackMotorOutput *= POWER_SCALE_FACTOR;

        hw.rf.setPower(normalizedFunction(rightFrontMotorOutput));
        hw.lf.setPower(normalizedFunction(leftFrontMotorOutput));
        hw.rb.setPower(normalizedFunction(rightBackMotorOutput));
        hw.lb.setPower(normalizedFunction(leftBackMotorOutput));
    }

    public static double normalizedFunction(double t) {
        double x = 128 * Math.abs(t);
        // Updated parameters: increased base (1.05), reduced linear term (0.1)
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

    // resets all motor encoders
    public void reset(){
        hw.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // motorProcess(sets the powers)
    public void motorProcess(){
        // combine main and adjustment velocities
        lfvel = lfVelMain + lfVelAdjustment1;
        lbvel = lbVelMain + lbVelAdjustment1;
        rfvel = rfVelMain + rfVelAdjustment1;
        rbvel = rbVelMain + rbVelAdjustment1;

        // limit velocities to maximum allowed
        double max = Math.max(Math.abs(lfvel), Math.max(Math.abs(lbvel), Math.max(Math.abs(rfvel), Math.abs(rbvel))));
        if (max > MAX_ANGULAR_VEL) {
            double scalar = MAX_ANGULAR_VEL / max;
            lfvel *= scalar;
            lbvel *= scalar;
            rfvel *= scalar;
            rbvel *= scalar;
        }

        // set motor velocities (radian/s) uses encoders to determine the velocity
        hw.rf.setVelocity(rfvel, AngleUnit.RADIANS);
        hw.lb.setVelocity(lbvel, AngleUnit.RADIANS);
        hw.rb.setVelocity(rbvel, AngleUnit.RADIANS);
        hw.lf.setVelocity(lfvel, AngleUnit.RADIANS);
    }
    // processes velocity control with no encoder feedback
    public void motorProcessNoEncoder(){
        double lfVelTemp = lfVelMain + lfVelAdjustment1;
        double lbVelTemp = lbVelMain + lbVelAdjustment1;
        double rfVelTemp = rfVelMain + rfVelAdjustment1;
        double rbVelTemp = rbVelMain + rbVelAdjustment1;

        // normalize vectors (between 0 to 1)
        double max = maxDouble( Math.abs(lfVelTemp), Math.abs(lbVelTemp), Math.abs(rfVelTemp), Math.abs(rbVelTemp) );
        //double max = Collections.max(Arrays.asList(tempVels));
        if (max > 1) {
            lfVelTemp /= max;
            lbVelTemp /= max;
            rfVelTemp /= max;
            rbVelTemp /= max;
        }

        lfvel = lfVelTemp;
        lbvel = lbVelTemp;
        rfvel = rfVelTemp;
        rbvel = rbVelTemp;

        // set motor powers

        setPowers(rfvel,lbvel,rbvel,lfvel);
    }

    //    named maxDouble temporarily to avoid name conflicts with local variable
    private double maxDouble(double... nums) {
        double max = -Double.MAX_VALUE;
        for (double num : nums) {
            max = Math.max(max, num);
        }
        return max;
    }

    //
    public void partialMove(double verticalVel, double horizontalVel, double rotationalVel){

        rbVelMain = (verticalVel * Math.cos(Math.toRadians(45)) + horizontalVel * Math.sin(Math.toRadians(45)) + rotationalVel * Math.sin(Math.toRadians(45)))*(1.41421356237);
        rfVelMain = (-horizontalVel * Math.cos(Math.toRadians(45)) + verticalVel * Math.sin(Math.toRadians(45)) + rotationalVel * Math.sin(Math.toRadians(45)))*(1.41421356237);
        lfVelMain = (verticalVel * Math.cos(Math.toRadians(45)) + horizontalVel * Math.sin(Math.toRadians(45)) - rotationalVel * Math.sin(Math.toRadians(45)))*(1.41421356237);
        lbVelMain = (-horizontalVel * Math.cos(Math.toRadians(45)) + verticalVel * Math.sin(Math.toRadians(45)) - rotationalVel * Math.sin(Math.toRadians(45)))*(1.41421356237);

    }

    //PartialMoveAdjustment is used in GridAutoCentering, it allows the robot to auto center to the grid
    public void partialMoveAdjustment(double verticalVel, double horizontalVel, double rotationalVel){

        rbVelAdjustment1 = (verticalVel * Math.cos(Math.toRadians(45)) + horizontalVel * Math.sin(Math.toRadians(45)) + rotationalVel * Math.sin(Math.toRadians(45)))*(1.41421356237);
        rfVelAdjustment1 = (-horizontalVel * Math.cos(Math.toRadians(45)) + verticalVel * Math.sin(Math.toRadians(45)) + rotationalVel * Math.sin(Math.toRadians(45)))*(1.41421356237);
        lfVelAdjustment1 = (verticalVel * Math.cos(Math.toRadians(45)) + horizontalVel * Math.sin(Math.toRadians(45)) - rotationalVel * Math.sin(Math.toRadians(45)))*(1.41421356237);
        lbVelAdjustment1 = (-horizontalVel * Math.cos(Math.toRadians(45)) + verticalVel * Math.sin(Math.toRadians(45)) - rotationalVel * Math.sin(Math.toRadians(45)))*(1.41421356237);

    }

    public void turnOffInternalPID() {
        hw.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Basic Mecanum robot movement
    public void move(double vertical, double horizontal, double rotational){

        rightFrontMotorOutput = (-horizontal * Math.cos(Math.toRadians(45)) + vertical * Math.sin(Math.toRadians(45)) + rotational * Math.sin(Math.toRadians(45)))*(1.41421356237);
        leftFrontMotorOutput = (vertical * Math.cos(Math.toRadians(45)) + horizontal * Math.sin(Math.toRadians(45)) - rotational * Math.sin(Math.toRadians(45)))*(1.41421356237);
        rightBackMotorOutput = (vertical * Math.cos(Math.toRadians(45)) + horizontal * Math.sin(Math.toRadians(45)) + rotational * Math.sin(Math.toRadians(45)))*(1.41421356237);
        leftBackMotorOutput = (-horizontal * Math.cos(Math.toRadians(45)) + vertical * Math.sin(Math.toRadians(45)) - rotational * Math.sin(Math.toRadians(45)))*(1.41421356237);

        setPowers(rightBackMotorOutput,leftBackMotorOutput,rightFrontMotorOutput,leftFrontMotorOutput);

    }

    public void moveToPosition(double power, double degree, int position){

        double y1 = power * Math.sin(Math.toRadians(degree)) * Math.cos(Math.toRadians(45)) - power * Math.cos(Math.toRadians(degree)) * Math.sin(Math.toRadians(45));
        double x1 = power * Math.cos(Math.toRadians(degree)) * Math.cos(Math.toRadians(45)) + power * Math.sin(Math.toRadians(degree)) * Math.sin(Math.toRadians(45));
        double y2 = power * Math.sin(Math.toRadians(degree)) * Math.cos(Math.toRadians(45)) - power * Math.cos(Math.toRadians(degree)) * Math.sin(Math.toRadians(45));
        double x2 = power * Math.cos(Math.toRadians(degree)) * Math.cos(Math.toRadians(45)) + power * Math.sin(Math.toRadians(degree)) * Math.sin(Math.toRadians(45));
        while (hw.rf.getCurrentPosition()<position){
            setPowers(0,0,0,0);

            setPowers(0,0,0,0);
        }
    }

    //Update PID controllers with new constants
    public void updatePIDConstants(){
        globalXController.setConstant(kpx, kdx, kix);
        globalYController.setConstant(kpy, kdy, kiy);
        globalThetaController.setConstant(kptheta, kdtheta, kitheta);
    }

    // Output Positional Getters for PID controllers
    public double globalXControllerOutputPositional(double XsetPoint, double Xfeedback) {
        return globalXController.outputPositional(XsetPoint, Xfeedback);
    }
    public double globalYControllerOutputPositional(double YsetPoint, double Yfeedback) {
        return globalYController.outputPositional(YsetPoint, Yfeedback);
    }
    public double globalThetaControllerOutputPositional(double ThetasetPoint, double Thetafeedback) {
        return globalThetaController.outputPositional(ThetasetPoint, Thetafeedback);
    }

    // stop all motors
    public void stop(){

            setPowers(0,0,0,0);

    }

    // TeleOp functions
    // movement control
    // input x - side to side (-1 to 1)
    // input y - front to back (-1 to 1)
    // input z - rotation (-1 to 1)
    // input theta - current heading (radians)
    public void fieldOrientedMove(double x, double y, double z, double theta) {
        // translate the field relative movement (joystick) into the robot relative movement
        //changed all 3 lines below
        double newX = x * Math.cos(theta) - y * Math.sin(theta);
        double newY = x * Math.sin(theta) + y * Math.cos(theta);

        // wheel power calculations
        rightFrontMotorOutput = - newY + newX - z;
        leftFrontMotorOutput = newY + newX + z;
        rightBackMotorOutput = newY + newX -  z;
        leftBackMotorOutput = - newY + newX + z;

        // normalize powers to maintain ratio while staying within the range of -1 to 1
        double largest = Math.max(
                Math.max(Math.abs(rightFrontMotorOutput), Math.abs(leftFrontMotorOutput)),
                Math.max(Math.abs(rightBackMotorOutput), Math.abs(leftBackMotorOutput)));

        if (largest > 1) {
            rightFrontMotorOutput /= largest;
            leftFrontMotorOutput /= largest;
            rightBackMotorOutput /= largest;
            leftBackMotorOutput /= largest;
        }

        // apply scaling and set motor powers
        rightFrontMotorOutput *= POWER_SCALE_FACTOR;
        leftFrontMotorOutput *= POWER_SCALE_FACTOR;
        rightBackMotorOutput *= POWER_SCALE_FACTOR;
        leftBackMotorOutput *= POWER_SCALE_FACTOR;

        setPowers(rightFrontMotorOutput,leftBackMotorOutput,rightBackMotorOutput,leftFrontMotorOutput);
    }

    public void setPowers (double x1, double x2, double y1, double y2){
        hw.rf.setPower(x1);
        hw.lb.setPower(x2);
        hw.rb.setPower(y1);
        hw.lf.setPower(y2);
    }
}

