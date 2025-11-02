package org.firstinspires.ftc.teamcode.subsystems.cameras;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class LogitechSubsystem {
    private Hardware hw;
    private static final boolean USE_WEBCAM = true;

    private static boolean GPP = false;
    private static boolean PGP = false;
    private static boolean PPG = false;

    private static String obelisk;

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    public void intiAprilTag(Hardware hw){
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

//        builder.setCamera(hw.logitech);

        builder.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }
    public void pattern(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && (detection.id == 21 || detection.id == 22 || detection.id == 23)) {
                obelisk = String.format(detection.metadata.name, 0, 3);
            }
        }

        if (obelisk == "PPG"){

        } else if (obelisk == "PPG"){

        }
    }
    public void telemetryAprilTag(Telemetry telemetry){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("%s", detection.metadata.name));
            } else {
                telemetry.addLine("No detection");
            }
        }
    }
}