package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@Autonomous(name = "AprilTag Test (Webcam Mimics Limelight)", group = "Vision")
public class logitrack extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private DcMotor turret;

    @Override
    public void runOpMode() {

        turret = hardwareMap.get(DcMotor.class, "LLmotor");

        // Initialize AprilTag detection
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(800, 448))
                .addProcessor(aprilTagProcessor)
                .build();

        telemetry.addLine("Camera initializing...");
        telemetry.update();

        // Wait for camera to start
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING && !isStopRequested()) {
            sleep(20);
        }

        telemetry.addLine("Camera ready! Press START.");
        telemetry.update();

        waitForStart();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            if (detections.isEmpty()) {
                telemetry.addLine("No AprilTag detected");
                turret.setPower(0);
            } else {
                // Just take the first tag for simplicity
                AprilTagDetection det = detections.get(0);
                double tx = det.ftcPose.x; // left/right offset in inches (can be treated similar to Limelight tx)
                double ty = det.ftcPose.y;

                // Mimic Limelight-style turret control
                if (tx > 2) { // Adjust thresholds for your camera/field distance
                    turret.setPower(0.1);
                } else if (tx < -2) {
                    turret.setPower(-0.1);
                } else {
                    turret.setPower(0);
                }

                telemetry.addData("Tag ID", det.id);
                telemetry.addData("Target X (in)", "%.1f", tx);
                telemetry.addData("Target Y (in)", "%.1f", ty);
                telemetry.addData("Turret Power", turret.getPower());
            }

            telemetry.update();
            sleep(20);
        }

        visionPortal.close();
    }
}