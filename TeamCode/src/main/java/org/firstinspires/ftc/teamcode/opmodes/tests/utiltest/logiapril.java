package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;

import android.annotation.SuppressLint;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "AprilTag Camera Overlay (Fixed Exposure)", group = "Vision")
public class logiapril extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private static final boolean USE_WEBCAM = true;
    public static final int VIEW_WIDTH = 864;
    public static final int VIEW_HEIGHT = 480;

    @Override
    public void runOpMode() {
        initAprilTagAndOverlay();


        if (USE_WEBCAM) {
            setManualExposure(300, 300); // Adjust these values
        }

        telemetry.addData("Camera", "Preview on DS + Dashboard");
        telemetry.addData(">", "Press START to run");
        telemetry.update();

        waitForStart();

        FtcDashboard dashboard = FtcDashboard.getInstance();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            telemetry.addData("Detections Count", detections.size());

            if (detections.isEmpty()) {
                telemetry.addLine("No AprilTag detected");
                dashboard.getTelemetry().addLine("No AprilTag detected");
            } else {
                for (AprilTagDetection det : detections) {
                    telemetry.addData("ID", det.id);
                    telemetry.addData("Decision Margin", "%.2f", det.decisionMargin);
                    if (det.ftcPose != null) {
                        telemetry.addData("X (in)", "%.1f", det.ftcPose.x);
                        telemetry.addData("Y (in)", "%.1f", det.ftcPose.y);
                        telemetry.addData("Z (in)", "%.1f", det.ftcPose.z);
                    }
                }
            }

            telemetry.update();
            dashboard.getTelemetry().update();

            sleep(20);
        }

        try {
            FtcDashboard.getInstance().stopCameraStream();
        } catch (Exception ignored) {}
    }

    private void initAprilTagAndOverlay() {
        // Set lens intrinsics for pose estimation (calibrate these values!)
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506) // Update with calibrated values
                .build();

        VisionProcessor overlayProcessor = new VisionProcessor() {
            @Override
            public void init(int width, int height, CameraCalibration calibration) {}

            @SuppressLint("DefaultLocale")
            @Override
            public Object processFrame(Mat frame, long captureTimeNanos) {
                List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
                for (AprilTagDetection det : detections) {
                    if (det.ftcPose != null) {
                        String text = String.format("X: %.1f Y: %.1f Z: %.1f",
                                det.ftcPose.x, det.ftcPose.y, det.ftcPose.z);
                        Point corner = new Point(det.corners[0].x, det.corners[0].y - 10);
                        Imgproc.putText(frame, text, corner, Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(0, 255, 0), 2);
                    }
                }
                return null;
            }

            @Override
            public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
        };

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTagProcessor)
                    .addProcessor(overlayProcessor)
                    .setCameraResolution(new Size(800, 448)) // Match calibration
                    .build();
        }

        // Wait for streaming to start
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING && !isStopRequested()) {
            sleep(20);
        }

        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);

        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING && !isStopRequested()) {
            sleep(20);
        }
        try {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);

            telemetry.addData("Exposure", "Set to %dms gain %d", exposureMS, gain);
        } catch (Exception e) {
            telemetry.addData("Exposure", "Failed to set (not supported?)");
        }
        telemetry.update();
    }
}
