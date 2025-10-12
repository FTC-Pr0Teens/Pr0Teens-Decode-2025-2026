package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="AprilTag Test Limelight", group="Vision")
public class AprilTagCameraTest extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor turret;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(6);
        turret = hardwareMap.get(DcMotor.class, "LLmotor");



        waitForStart();

        limelight.start();

        while (opModeIsActive()) {

            limelight.start();


            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy();
                if (tx > 5) {
                    turret.setPower(0.1);
                } else if (tx < -5) {
                    turret.setPower(-0.1);
                } else {
                    turret.setPower(0);
                }// How far up or down the target is (degrees)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
            }
            else {
                telemetry.addData("Limelight", "No Targets");
            }
        }
    }
}