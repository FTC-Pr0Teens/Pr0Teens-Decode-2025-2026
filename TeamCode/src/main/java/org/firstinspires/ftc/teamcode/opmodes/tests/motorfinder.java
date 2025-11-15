package org.firstinspires.ftc.teamcode.opmodes.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name = "motor finder")
public class motorfinder extends LinearOpMode {

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

            hw.lb.setDirection(DcMotorSimple.Direction.REVERSE);
            hw.lf.setDirection(DcMotorSimple.Direction.REVERSE);
            hw.rf.setDirection(DcMotorSimple.Direction.FORWARD);
            hw.rb.setDirection(DcMotorSimple.Direction.FORWARD);


            waitForStart();


            while (opModeIsActive()) {
                if (gamepad1.a) {
                    hw.lf.setPower(0.5); //
                } else {
                    hw.lf.setPower(0);
                }
                if (gamepad1.b) {
                    hw.lb.setPower(0.5); //
                } else {
                    hw.lf.setPower(0);
                }
                if (gamepad1.x) {
                    hw.rf.setPower(0.5); //
                } else {
                    hw.lf.setPower(0);
                }
                if (gamepad1.y) {
                    hw.rb.setPower(0.5); //
                } else {
                    hw.lf.setPower(0);
                }
            }
        }
}
