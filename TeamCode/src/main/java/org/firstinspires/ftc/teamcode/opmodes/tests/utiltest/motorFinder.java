package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware;

@Autonomous(name = "motor finder")
public class motorFinder extends LinearOpMode {
    private Hardware hw;
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;

    public void runOpMode() throws InterruptedException {
        hw = Hardware.getInstance(hardwareMap);
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftRear = hardwareMap.get(DcMotor.class, "lb");
        rightRear = hardwareMap.get(DcMotor.class, "rb");

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();


        while (opModeIsActive()) {
//            if (gamepad1.a) {
                hw.lf.setPower(0.5);
//            }
//            if (gamepad1.b) {
                hw.lb.setPower(0.5);
//            }
//            if (gamepad1.x) {
                hw.rf.setPower(0.5);
//            }
//            if (gamepad1.y) {
                hw.rb.setPower(0.5);
//            }


        }
    }
}
