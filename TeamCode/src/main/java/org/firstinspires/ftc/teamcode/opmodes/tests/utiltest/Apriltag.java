///* Copyright (c) 2024 Dryw Wade. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.List;
//
//@TeleOp(name = "AprilTag Localization Red", group = "Concept")
//
//public class Apriltag extends LinearOpMode {
//
//    private static final boolean USE_WEBCAM = true;
//
//    private static boolean GPP = false;
//    private static boolean PGP = false;
//    private static boolean PPG = false;
//
//    private static int obelisk;
////    public CRServo turret;
//    private Position cameraPosition = new Position(DistanceUnit.INCH,
//            0, 0, 0, 0);
//    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
//            0, -90, 0, 0);
//
//    private AprilTagProcessor aprilTag;
//
//    private VisionPortal visionPortal;
//
//    @Override
//    public void runOpMode() {
//
//        initAprilTag();
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            telemetryAprilTag();
//
//            if (GPP == true) {
//                obelisk = 21;
//                telemetry.addLine("GPP Detected");
//            } else if (PGP == true) {
//                obelisk = 22;
//                telemetry.addLine("PGP Detected");
//            } else if (PPG == true) {
//                obelisk = 23;
//                telemetry.addLine("PPG Detected");
//            } else {
////                turret.setPower(0.1);
//            }
//
//            // Push telemetry to the Driver Station.
//            telemetry.update();
//
//        }
//
//        // Save more CPU resources when camera is no longer needed.
//        visionPortal.close();
//
//    }
//
//    /**
//     * Initialize the AprilTag processor.
//     */
//    private void initAprilTag() {
//
//        // Create the AprilTag processor.
//        aprilTag = new AprilTagProcessor.Builder()
//                .setDrawTagOutline(true)
//                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//                .setCameraPose(cameraPosition, cameraOrientation)
//
//                .build();
//
//        // Create the vision portal by using a builder.
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//
//        builder.setAutoStopLiveView(true);
//
//        // Set and enable the processor.
//        builder.addProcessor(aprilTag);
//
//        // Build the Vision Portal, using the above settings.
//        visionPortal = builder.build();
//
//        //visionPortal.setProcessorEnabled(aprilTag, true);
//
//    }
//
//    private void telemetryAprilTag() {
//
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                if (detection.id == 21) {
//                    GPP = true;
//                }
//                if (detection.id == 22) {
//                    PGP = true;
//                }
//                if (detection.id == 23) {
//                    PPG = true;
//                }
//
////                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("%s", detection.metadata.name));
////                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
////                            detection.robotPose.getPosition().x,
////                            detection.robotPose.getPosition().y,
////                            detection.robotPose.getPosition().z));
////                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
////                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
////                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
////                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
//
//            } else {
//                telemetry.addLine("No detection");
//            }
//        }
//
////        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
////        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//
//    }
//}

/* Copyright (c) 2024 Dryw Wade. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//@TeleOp(name = "AprilTag Localization Red", group = "Concept")

public class Apriltag extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;

    private static boolean GPP = false;
    private static boolean PGP = false;
    private static boolean PPG = false;

    private static String obelisk;
    //    public CRServo turret;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initAprilTag();
        waitForStart();

        while (opModeIsActive()) {

            telemetryAprilTag();

            if (GPP == true) {
                telemetry.addLine(obelisk.substring(0, 3));
            } else if (PGP == true) {
                telemetry.addLine("PGP Detected");
            } else if (PPG == true) {
                telemetry.addLine("PPG Detected");
            } else {
//                turret.setPower(0.1);
            }

            // Push telemetry to the Driver Station.
            telemetry.update();

        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        builder.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        //visionPortal.setProcessorEnabled(aprilTag, true);

    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == 21) {
                    GPP = true;
                }
                if (detection.id == 22) {
                    PGP = true;
                }
                if (detection.id == 23) {
                    PPG = true;
                }

//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("%s", detection.metadata.name));
                obelisk = detection.metadata.name;
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
//                            detection.robotPose.getPosition().x,
//                            detection.robotPose.getPosition().y,
//                            detection.robotPose.getPosition().z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
//                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
//                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
//                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));

            } else {
                telemetry.addLine("No detection");
            }
        }

//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }
}
