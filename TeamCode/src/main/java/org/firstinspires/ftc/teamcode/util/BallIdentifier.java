package org.firstinspires.ftc.teamcode.util;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

@TeleOp(name = "Center Ball", group = "Concept")
public class BallIdentifier extends LinearOpMode {

    DcMotor rd, ld;

    @Override
    public void runOpMode() {
        rd = hardwareMap.get(DcMotor.class, "rdrive");
        ld = hardwareMap.get(DcMotor.class, "ldrive");

        ld.setDirection(DcMotor.Direction.REVERSE);
        rd.setDirection(DcMotor.Direction.FORWARD);

        // Create two blob processors: purple + green
        ColorBlobLocatorProcessor purpleLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();

        ColorBlobLocatorProcessor greenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();

        // Vision portal with both processors
        VisionPortal portal = new VisionPortal.Builder()
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(purpleLocator)
                .addProcessor(greenLocator)
                .build();

        int frameCenterX = 160; // 320px / 2
        waitForStart();

        while (opModeIsActive()) {
            // Get blobs from both locators
            List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleLocator.getBlobs();
            List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenLocator.getBlobs();

            Circle targetCircle = null;

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    1000, 20000, purpleBlobs);  // filter out very small blobs.

//            ColorBlobLocatorProcessor.Util.filterByCriteria(
//                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
//                    0.6, 1, purpleBlobs);

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    1000, 20000, greenBlobs);  // filter out very small blobs.

//            ColorBlobLocatorProcessor.Util.filterByCriteria(
//                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
//                    0.6, 1, greenBlobs);

            if (!purpleBlobs.isEmpty() && !greenBlobs.isEmpty()) {
                Circle purpleCircle = purpleBlobs.get(0).getCircle();
                Circle greenCircle  = greenBlobs.get(0).getCircle();

                if (purpleCircle.getRadius() > greenCircle.getRadius()) {
                    targetCircle = purpleCircle;
                } else {
                    targetCircle = greenCircle;
                }

            } else if (!purpleBlobs.isEmpty()) {
                targetCircle = purpleBlobs.get(0).getCircle();
            } else if (!greenBlobs.isEmpty()) {
                targetCircle = greenBlobs.get(0).getCircle();
            }

            if (targetCircle != null) {
                double ballX = targetCircle.getX();
                double radius = targetCircle.getRadius();

                double errorX = ballX - frameCenterX;
                double turnPower = errorX * 0.008; // tuning constant
                double forwardPower = 0.6;

                if (radius > 80) {
                    forwardPower = 0.0;
                    turnPower = 0.0;
                }

                double leftPower  = forwardPower + turnPower;
                double rightPower = forwardPower - turnPower;


                ld.setPower(leftPower);
                rd.setPower(rightPower);

                telemetry.addData("Target X", ballX);
                telemetry.addData("Radius", radius);
                telemetry.addData("TurnPower", turnPower);
                telemetry.addData("Error: ", errorX);

            } else {

                ld.setPower(0);
                rd.setPower(0);

                telemetry.addData("Target", "none");
            }

            telemetry.update();
        }
    }
}
