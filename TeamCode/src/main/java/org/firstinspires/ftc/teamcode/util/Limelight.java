package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp(name = "Center April", group = "Concept")
public class Limelight extends LinearOpMode {

    DcMotor rd, ld;
    Limelight3A limelight;

    @Override
    public void runOpMode() {
        rd = hardwareMap.get(DcMotor.class, "rb");
        ld = hardwareMap.get(DcMotor.class, "lb");

        ld.setDirection(DcMotor.Direction.REVERSE);
        rd.setDirection(DcMotor.Direction.FORWARD);


        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        limelight.pipelineSwitch(6);

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();

            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("ta", ta);

            if (result.isValid()) {
                // Get horizontal offset (tx), vertical offset (ty), area (ta)
//                double tx = result.getTx();   // left/right offset (deg)
//                double ty = result.getTy();   // up/down offset (deg)
//                double ta = result.getTa();   // target area (proxy for distance)

                // Control math
                double errorX = tx;                  // 0 = centered
                double turnPower = errorX * 0.02;    // tune this constant
                double forwardPower = 0.3;

                if (ta > 30.0) {
                    forwardPower = 0.0;
                    turnPower = 0.0;
                }

                double leftPower  = forwardPower + turnPower;
                double rightPower = forwardPower - turnPower;

                ld.setPower(leftPower);
                rd.setPower(rightPower);

//                telemetry.addData("tx", tx);
//                telemetry.addData("ty", ty);
//                telemetry.addData("ta", ta);
                telemetry.addData("ErrorX", errorX);
                telemetry.addData("ForwardPower", forwardPower);
                telemetry.addData("LeftPower", leftPower);
                telemetry.addData("RightPower", rightPower);
                telemetry.addData("TurnPower", turnPower);
            } else {
                // No target detected
                ld.setPower(0);
                rd.setPower(0);
                telemetry.addLine("No target");
            }

            telemetry.update();
        }
    }
}
