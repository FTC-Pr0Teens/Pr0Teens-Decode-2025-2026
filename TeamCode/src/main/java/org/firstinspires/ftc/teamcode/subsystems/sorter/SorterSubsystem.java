package org.firstinspires.ftc.teamcode.subsystems.sorter;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

import java.util.ArrayList;

public class SorterSubsystem {
    ArrayList<String> order;
    ArrayList<String> intake;
    ColorSensor colourSensor;
    private Servo sorter;
    private Servo pusher;
    private DcMotor rb;
    String firstColour;
    String secondColour;
    public int index;
    int length;
    int purpleNum;
    int greenNum;
    private Hardware hw;
    private ElapsedTime timer;

    public Telemetry telemetry;

    double firstPos;
    double secondPos;
    double thirdPos;



    public SorterSubsystem(Hardware hw, ArrayList<String>  motif, Telemetry telemetry){
        this.hw = hw;
        this.colourSensor = hw.colourSensor;
        this.sorter = hw.sorter;
        this.pusher = hw.pusher;
        this.rb = hw.rb;
        this.order = motif;
        timer = new ElapsedTime();
        length = intake.size();
        intake = new ArrayList<>();
        this.telemetry = telemetry;
        firstPos = 0;
        secondPos = 0.067;
        thirdPos = 0.133;

    }

    public void detectColour() { //change name to intake
        telemetry.addLine("detectColour");
        telemetry.update();
        int red = colourSensor.red();
        int green = colourSensor.green();
        int blue = colourSensor.blue();
        int alpha = colourSensor.alpha();

        int length = intake.size();

        telemetry.addData("intake: ", intake.size());

        telemetry.addData("Red", colourSensor.red());
        telemetry.addData("Green", colourSensor.green());
        telemetry.addData("Blue", colourSensor.blue());
        telemetry.addData("Alpha", colourSensor.alpha());
        telemetry.update();
        //purple
        if (length < 3) {
            CheckArtifact();
            if (blue > green) { //
                telemetry.addLine("purple detected");
                telemetry.update();
                if (length == 0) {
                    sorter.setPosition(firstPos);
                    waitTime(1);
                    intake.add("purple");
                    telemetry.addLine(Integer.toString(length));
                    telemetry.update();
                } else if (length == 1) {
                    sorter.setPosition(secondPos);
                    waitTime(1);
                    sorter.setPosition(firstPos);
                    waitTime(1);
                    intake.add("purple");
                    telemetry.addLine(Integer.toString(length));
                    telemetry.update();
                } else {
                    sorter.setPosition(thirdPos);
                    waitTime(1);
                    sorter.setPosition(firstPos);
                    waitTime(1);
                    intake.add("purple");
                    telemetry.addLine(Integer.toString(length));
                    telemetry.update();
                }
            }
        }

        if (green > blue) {
            telemetry.addData("Green", colourSensor.green());
            telemetry.update();
            telemetry.addLine("green detected");
            if (length == 0) {
                sorter.setPosition(firstPos);
                waitTime(1);
                intake.add("green");
                telemetry.addLine(Integer.toString(length));
                telemetry.update();
            } else if (length == 1) {
                sorter.setPosition(secondPos);
                waitTime(1);
                sorter.setPosition(firstPos);
                waitTime(1);
                intake.add("green");
                telemetry.addLine(Integer.toString(length));
                telemetry.update();
            } else {
                sorter.setPosition(thirdPos);
                waitTime(1);
                sorter.setPosition(firstPos);
                waitTime(1);
                intake.add("green");
                telemetry.addLine(Integer.toString(length));
                telemetry.update();
            }
        }
    }

    public void sort() { //change name to outake
        int length = intake.size();
        index = intake.indexOf(new String("green"));
        telemetry.addData("index ", index);
        telemetry.update();
        sorter.setPosition(firstPos);
        firstColour = order.get(0);
        secondColour = order.get(1);
        //120 degrees = 0.067
        //240 degrees = 0.133
        if (firstColour.equals("purple") && secondColour.equals("purple")) {
            if (index == 0) {
                sorter.setPosition(secondPos);
                waitTime(1);
                sorter.setPosition(thirdPos);
                waitTime(1);
                sorter.setPosition(firstPos);
                waitTime(1);
                intake.clear();
            } else if (index == 1) {
                sorter.setPosition(firstPos);//60 degrees
                waitTime(1);
                sorter.setPosition(secondPos);
                waitTime(1);
                sorter.setPosition(thirdPos);
                waitTime(1);
                sorter.setPosition(firstPos);
                waitTime(1);
                intake.clear();

            } else if (index == 2) {
                sorter.setPosition(firstPos);//60 degrees
                waitTime(1);
                sorter.setPosition(thirdPos);
                waitTime(1);
                sorter.setPosition(secondPos);
                waitTime(1);
                sorter.setPosition(firstPos);
                waitTime(1);
                intake.clear();
            }
        } else if (firstColour.equals("purple") && secondColour.equals("green")) {
            if (index == 0) {
                sorter.setPosition(secondPos);
                waitTime(1);
                sorter.setPosition(firstPos);
                waitTime(1);
                sorter.setPosition(thirdPos);
                waitTime(1);
                sorter.setPosition(firstPos);
                waitTime(1);
                intake.clear();
            } else if (index == 1) {
                sorter.setPosition(firstPos);
                waitTime(1);
                sorter.setPosition(thirdPos);
                waitTime(1);
                sorter.setPosition(secondPos);
                waitTime(1);
                sorter.setPosition(firstPos);
                waitTime(1);
                intake.clear();
            } else if (index == 2) {
                sorter.setPosition(firstPos);
                waitTime(1);
                sorter.setPosition(secondPos);
                waitTime(1);
                sorter.setPosition(thirdPos);
                waitTime(1);
                sorter.setPosition(firstPos);
                waitTime(1);
                intake.clear();
            }
        } else { //if (firstColour.equals("green")) {
            telemetry.addData("index", index);
            telemetry.update();
            if (index == 0) {
                sorter.setPosition(firstPos);//60 degrees
                waitTime(1);
                sorter.setPosition(secondPos);
                waitTime(1);
                sorter.setPosition(thirdPos);
                waitTime(1);
                sorter.setPosition(firstPos);
                waitTime(1);
                intake.clear();
            } else if (index == 1) {
                sorter.setPosition(thirdPos);//60 desgrees
                waitTime(1);
                sorter.setPosition(secondPos);
                waitTime(1);
                sorter.setPosition(firstPos);
                waitTime(1);
                intake.clear();
            } else if (index == 2) {
                sorter.setPosition(secondPos);//60 degrees
                waitTime(1);
                sorter.setPosition(thirdPos);
                waitTime(1);
                sorter.setPosition(firstPos);
                waitTime(1);
                intake.clear();
            }
        }

    }

    void CheckArtifact () {
        purpleNum = 0;
        greenNum = 0;
        for (int pointer = 0; pointer < 3; pointer++) {
            if (intake.get(pointer).equalsIgnoreCase("purple")) {
                purpleNum++;
                if(purpleNum == 2){
                    telemetry.addLine("purple artifacts");
                }
            }
            telemetry.addLine("# of Purples: " + purpleNum);
            telemetry.update();
        }

        for (int check = 0; check < 3; check++) {
            if (intake.get(check).equalsIgnoreCase("purple")) {
                greenNum++;
                if(greenNum == 2){
                    telemetry.addLine("already have 1 green artifacts");
                }
            }
            telemetry.addLine("# of Green: " + greenNum);
            telemetry.update();
        }

    }
    public void push () {
        pusher.setPosition(0.1);
        waitTime(1);
        pusher.setPosition(0);
    }
    public void waitTime(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        // Wait while OpMode is still active and time hasn't elapsed
        while (timer.seconds() < seconds) {
            // Optionally do telemetry updates
            telemetry.addData("Waiting", "%.1f / %.1f", timer.seconds(), seconds);
            telemetry.update();
        }
    }
    public void setLength(){
            length = intake.size();
            telemetry.addLine(Integer.toString(length));
        }
        public void clearList(){
        intake.clear();
        }


}
