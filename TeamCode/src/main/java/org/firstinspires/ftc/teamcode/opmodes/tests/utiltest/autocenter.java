package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="AprilTag Follow (Robot Rotate)", group="Vision")
public class autocenter extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(6);

        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "lf");
        frontRight = hardwareMap.get(DcMotor.class, "rf");
        backLeft = hardwareMap.get(DcMotor.class, "lb");
        backRight = hardwareMap.get(DcMotor.class, "rb");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();

        limelight.start();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s", status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx(); // Horizontal offset in degrees
                double ty = result.getTy(); // Vertical offset

                double turnPower = 0.0;

                // Simple proportional control (turn toward tag)
                if (tx > 3) {
                    turnPower = 0.15;  // turn right
                } else if (tx < -3) {
                    turnPower = -0.15; // turn left
                } else {
                    turnPower = 0.0;   // centered
                }

                // Apply turn to all motors
                frontLeft.setPower(turnPower);
                backLeft.setPower(turnPower);
                frontRight.setPower(-turnPower);
                backRight.setPower(-turnPower);

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Turn Power", turnPower);
            } else {
                // Stop moving if no tag detected
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);

                telemetry.addData("Limelight", "No Targets");
            }

            telemetry.update();
        }
    }
}