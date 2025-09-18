package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Hardware;


import java.util.List;
@TeleOp(name = "please work limelight")

public class AprilTagCameraTest extends LinearOpMode {

    Limelight3A limelight;
    private TelemetryPacket packet;
    private FtcDashboard dash;
    double limelightMountAngleDegrees = 25.0;   // Tilt of the limelight
    double limelightLensHeightInches = 20.0;    // From ground to lens
    double goalHeightInches = 60.0;



    @Override


    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Hardware hw = new Hardware(hardwareMap);


        waitForStart();
        while (opModeIsActive()) {

            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();

//            Pose3D botpose = result.getBotpose();
//            double captureLatency = result.getCaptureLatency();
//            double targetingLatency = result.getTargetingLatency();
//            double parseLatency = result.getParseLatency();


            telemetry.addData("tx", result.getTx());
            telemetry.addData("txnc", result.getTxNC()); //Horizontal Offset From Principal Pixel To Target (degrees)
            telemetry.addData("ty", result.getTy());
            telemetry.addData("tync", result.getTyNC()); //	Vertical Offset From Principal Pixel To Target (degrees)


            limelight.pipelineSwitch(6);
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.start();


            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                    int setPriorityID = 583;
                    if (fr.getFiducialId() == setPriorityID) {
                        // Use THIS tag’s info
                        telemetry.addData("Priority Tag ID", fr.getFiducialId());
                        telemetry.addData("X Offset", fr.getTargetXDegrees());
                        telemetry.addData("Y Offset", fr.getTargetYDegrees());
                    }


                    // ty = vertical offset angle (in degrees)
                    double targetOffsetAngle_Vertical = result.getTy();


                    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
                    double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

                    double distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

                    telemetry.addData("Target Seen", true);
                    telemetry.addData("Distance (inches)", distance);
                    telemetry.addData("ty", targetOffsetAngle_Vertical);
                }

                telemetry.update();
            }
        }


    }
}

