package org.firstinspires.ftc.teamcode.subsystems.camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Specifications;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class LimeLightSubsystem {
    private Hardware hw;
    private Limelight3A limelight;


    // Filter to smooth angle readings
    public MovingAverageFilter angle;

    // Corner positions of detected vision targets
    public List<Vec2> corners = new ArrayList<>();

    // Height of the camera mounted on the robot (in cm)
    private final double heightCamera = 19;
    private final double heightCamera2 = 0;

    // Calculated distances to target
    private double yDistance;
    private double xDistance;

    // Offset of the camera from robot center (in cm)
    private final double xOffset = 0;
    private double yOffset = 0;

    // Angle of the camera mounted on the robot (degrees)
    private double cameraAngle = 30;
    private double cameraAngle2 = 48; // Secondary camera angle

    // Counters to track detection and angle readings
    private int detectionCounter = 0;
    private int angleDetectionCounter = 0;

    // Angle to sample
    public Vec2 samplePose;


    List<Vec2> balls = new ArrayList<>();

    // Sets the camera angle (externally adjustable)
    public void setCameraAngle(double angle){
        cameraAngle = angle;
    }

    // Enum to represent team colors
    public enum COLOR {
        BLUE,
        RED,
        YELLOW
    }

    // Mapping from team color to pipeline index
    public HashMap<COLOR, Integer> pipelines = new HashMap<>();

    private COLOR TEAM_COLOR;


    public LimeLightSubsystem(Hardware hw) {
        this.hw = hw;
        angle = new MovingAverageFilter();
        pipelines.put(COLOR.YELLOW,3);


    }
    public void initCamera(){
        limelight.setPollRateHz(100);
        limelight.start();
    }


    // Start the camera
    public void start() {
        limelight.start();
    }

    // Stop the camera
    public void stop() {
        limelight.stop();
    }



    // Get current camera status
    public LLStatus getStatus(){
        LLStatus status = limelight.getStatus();
        return status;
    }


    public double getSnapscriptAngle(LLResult result){
        if (result == null){
            return 0;
        } else {
            angleDetectionCounter++;
            return angle.filterAngle(result.getPythonOutput()[3]);
        }
    }

    // Switch to neural network pipelines
    public void switchToYellowNeural(){ limelight.pipelineSwitch(6); }
    public void switchToBlueNeural(){ limelight.pipelineSwitch(7); }
    public void switchToRedNeural(){ limelight.pipelineSwitch(8); }

    private Vec2 sampleAngles;

    // Calculate sample position based on angle
    public Vec2 findSample(Vec2 sampleAngles){
        this.sampleAngles = sampleAngles;
        if (sampleAngles != null){
            yDistance = (heightCamera - 3.8) * Math.tan(Math.toRadians((cameraAngle + sampleAngles.y)));
            xDistance = (yDistance * Math.toRadians(sampleAngles.x));
            return new Vec2(xDistance - xOffset, yDistance - yOffset);
        } else {
            return new Vec2(0,0);
        }
    }

    // Directly find sample position using Limelight's output
    public Vec2 findSampleDirect(LLResult result){
        if (result != null){
            detectionCounter++;
            double verticalAngle = (90 - cameraAngle) + result.getTy();
            yDistance = (heightCamera - 3.8) * Math.tan(Math.toRadians(verticalAngle));
            double horizontalAngle = Math.tan(Math.toRadians(result.getTx()));
            xDistance = yDistance * horizontalAngle;
            return new Vec2(xDistance - xOffset, yDistance - yOffset);
        } else {
            return new Vec2(0,0);
        }
    }

    // Alternate calculation using secondary snapshot
    public Vec2 findSampleDirectSecondSnapshot(LLResult result){
        if (result != null){
            detectionCounter++;
            double verticalAngle = (90 - cameraAngle2) + result.getTy();
            yDistance = (heightCamera2 - 1.5) * Math.tan(Math.toRadians(verticalAngle));
            double horizontalAngle = Math.tan(Math.toRadians(result.getTx()));
            xDistance = yDistance * horizontalAngle;
            return new Vec2(xDistance - xOffset, yDistance - yOffset);
        } else {
            return new Vec2(0,0);
        }
    }

    // Get sample X and Y angles
    public double getSampleAnglesY(){ return sampleAngles.y; }
    public double getSampleAnglesX(){ return sampleAngles.x; }

    // Extract sample corner positions from vision result
    public List<Vec2> getSampleCornerPositions(LLResult analysis) {
        List<Vec2> corners = new ArrayList<>();
        if (analysis == null) return null;
        List<LLResultTypes.ColorResult> crs = analysis.getColorResults();
        if (crs.isEmpty()) return null;
        for (List<Double> positions: crs.get(0).getTargetCorners()) {
            corners.add(new Vec2(positions.get(0), positions.get(1)));
        }
        return corners;
    }

    double slope;
    double sideLength;
    double sampleCornerSize;


    public double getSlope(){ return slope; }
    public double getSampleCornerSize(){ return sampleCornerSize; }
    public double getSideLength(){ return sideLength; }

    public double getTx(LLResult analysis){ return analysis == null ? 0 : analysis.getTx(); }
    public double getTy(LLResult analysis){ return analysis == null ? 0 : analysis.getTy(); }


    public Vec2 getBallPosition(LLResult analysis) {
        if (analysis != null) {
            Vec2 position = new Vec2(analysis.getTx(), analysis.getTy());
            balls.add(position);
            if (balls.size() > Specifications.CVSmoothing) {
                balls.remove(0);
            }
            Vec2 average = new Vec2(0.0, 0.0);
            for (Vec2 sample : balls) {
                average.add(sample);
            }
            average.divide(Specifications.CVSmoothing);
            if (balls.size() >= Specifications.CVSmoothing) return new Vec2(average.x, average.y);
            else return null;
        } else {
            balls.clear();
        }
        return null;
    }

    // Get Limelight instance
    public Limelight3A getLimelight() { return limelight; }

    // Return detection counters
    public int getDetectionCounter(){ return detectionCounter; }
    public int getAngleDetectionCounter(){ return angleDetectionCounter; }
}



