package org.firstinspires.ftc.teamcode.subsystems.camera;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class LimelightSubsystem {
    private Hardware hw;
    private Limelight3A limelight;

    private double tx;
    private double ty;
    private double area;
    private String color;

    public LimelightSubsystem(Hardware hw, Telemetry telemetry){
        this.hw = hw;
        this.limelight = hw.limelight;

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(1);

        limelight.start();

        telemetry.addLine("Start ");
        telemetry.update();

    }

    public void ballPosition(){
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
            if (!detections.isEmpty()) {
                LLResultTypes.DetectorResult firstDetection = detections.get(0);
                tx = firstDetection.getTargetXPixels();
                ty = firstDetection.getTargetYPixels();
                area = firstDetection.getTargetArea();
                color = firstDetection.getClassName();
            } else {

            }
        }
    }

    public void telemetryLimelight(Telemetry telemetry){
        LLResult result = limelight.getLatestResult();

        if (result.isValid()) {
//            Pose3D botpose = result.getBotpose();
//            telemetry.addData("Botpose", botpose.toString());

            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
//            for (LLResultTypes.DetectorResult det : detections) {
//                telemetry.addData("tx ", det.getTargetXPixels());
//                telemetry.addData("ty ", det.getTargetYPixels());
//                telemetry.addData("class ", det.getClassName());
//                telemetry.addData("class id ", det.getClassId());
//            }

                LLResultTypes.DetectorResult firstDetection = detections.get(0);
                tx = firstDetection.getTargetXPixels();
                ty = firstDetection.getTargetYPixels();
                area = firstDetection.getTargetArea();
                color = firstDetection.getClassName();
                telemetry.addData("Detected Color", color);
                telemetry.addData("Horizontal Offset (tx)", "%.2f", tx);
                telemetry.addData("Vertical Offset (ty)", "%.2f", ty);
                telemetry.addData("Target Area", "%.2f", area);

        } else {
            telemetry.addData("Limelight", "No data available");
        }

        telemetry.update();
    }
}
