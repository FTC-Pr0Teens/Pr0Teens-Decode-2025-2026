package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp
public class MotorTest extends LinearOpMode {
    Hardware hw;
    @Override
    public void runOpMode() throws InterruptedException {
        hw = Hardware.getInstance(hardwareMap);
        hw.lb.setDirection(DcMotorSimple.Direction.FORWARD);
        hw.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.rf.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.rb.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                hw.lf.setPower(1);
            } else {
                hw.lf.setPower(0);
            }

            if (gamepad1.b) {
                hw.rf.setPower(1);
            } else {
                hw.rf.setPower(0);
            }

            if (gamepad1.x) {
                hw.lb.setPower(1);
            } else {
                hw.lb.setPower(0);
            }

            if (gamepad1.y) {
                hw.rb.setPower(1);
            } else {
                hw.rb.setPower(0);
            }
//            if (gamepad1.a) {
//                lf.setPower(gamepad1.a ? 1.0 : 0.0);  // A -> LF
//                rf.setPower(gamepad1.b ? 1.0 : 0.0);  // B -> RF
//                lb.setPower(gamepad1.x ? 1.0 : 0.0);  // X -> LB
//                rb.setPower(gamepad1.y ? 1.0 : 0.0);  // Y -> RB
//            }
            telemetry.addData("LF Power", hw.lf.getPower());
            telemetry.addData("RF Power", hw.rf.getPower());
            telemetry.addData("LB Power", hw.lb.getPower());
            telemetry.addData("RB Power", hw.rb.getPower());
            telemetry.update();
        }
    }
}