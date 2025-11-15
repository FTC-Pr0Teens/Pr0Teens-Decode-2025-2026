package org.firstinspires.ftc.teamcode.subsystems.sorting;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.cameras.LogitechSubsystem;

import java.util.ArrayList;

public class SortingSubsystem {
    ArrayList<String> motif;
    ArrayList<String> intake;
    ColorSensor colourSensor;
    private Servo sorter;
    private Servo pusher;
    private DcMotor rb;
    String firstColour;
    String secondColour;
    public int index;
    int length;
    int PNum;
    int GNum;
    private Hardware hw;

    private ElapsedTime timer;

    public Telemetry telemetry;
    private LogitechSubsystem logitechSubsystem;
    double firstPos;
    double secondPos;
    double thirdPos;
    double pos;

    public SortingSubsystem(Hardware hw, Telemetry telemetry, String motif) {
        logitechSubsystem = new LogitechSubsystem(hw, "red");
        this.hw = hw;
        //    this.colourSensor = hw.colourSensor;
        this.sorter = hw.sorter;
        this.pusher = hw.pusher;
        this.rb = hw.rb;
//         this.order = motif;
        timer = new ElapsedTime();
        intake = new ArrayList<>();
        this.motif = new ArrayList<>();

        //  length = 0;
        setMotif(motif);
        //motif;
        this.telemetry = telemetry;
        firstPos = 0;
        secondPos = 0.38; //0.42
        thirdPos = 0.78; //0.88
        pos = sorter.getPosition();
    }

    //organizes artifacts on intake and adds the colour of each artifact to a list in order of entry
    public void detectColour() { //change name to intake
        telemetry.addLine("detectColour");
        telemetry.update();
        int red = colourSensor.red();
        int green = colourSensor.green();
        int blue = colourSensor.blue();
        int alpha = colourSensor.alpha();

        length = intake.size();

        telemetry.addData("intake: ", intake.size());

        telemetry.addData("Red", colourSensor.red());
        telemetry.addData("G", colourSensor.green());
        telemetry.addData("Blue", colourSensor.blue());
        telemetry.addData("Alpha", colourSensor.alpha());
        telemetry.update();
        //P
        if (length < 3) {
            CheckArtifact();
            if (blue > green) { //
                telemetry.addLine("P detected");
                telemetry.update();
                if (length == 0) {
                    sorter.setPosition(firstPos);
                    waitTime(1);
                    intake.add("P");
                    telemetry.addLine(Integer.toString(length));
                    telemetry.update();
                } else if (length == 1) {
                    sorter.setPosition(secondPos);
                    waitTime(1);
                    sorter.setPosition(firstPos);
                    waitTime(1);
                    intake.add("P");
                    telemetry.addLine(Integer.toString(length));
                    telemetry.update();
                } else {
                    sorter.setPosition(thirdPos);
                    waitTime(1);
                    sorter.setPosition(firstPos);
                    waitTime(1);
                    intake.add("P");
                    telemetry.addLine(Integer.toString(length));
                    telemetry.update();
                }
            }
        }

        if (green > blue) {
            telemetry.addData("G", colourSensor.green());
            telemetry.update();
            telemetry.addLine("G detected");
            if (length == 0) {
                sorter.setPosition(firstPos);
                waitTime(1);
                intake.add("G");
                telemetry.addLine(Integer.toString(length));
                telemetry.update();
            } else if (length == 1) {
                sorter.setPosition(secondPos);
                waitTime(1);
                sorter.setPosition(firstPos);
                waitTime(1);
                intake.add("G");
                telemetry.addLine(Integer.toString(length));
                telemetry.update();
            } else {
                sorter.setPosition(thirdPos);
                waitTime(1);
                sorter.setPosition(firstPos);
                waitTime(1);
                intake.add("G");
                telemetry.addLine(Integer.toString(length));
                telemetry.update();
            }
        }
    }

    //organizes balls on outtake by comparing intake list with the motif
    public void sort() {
        length = intake.size();
        index = intake.indexOf(new String("G"));
        telemetry.addData("index ", index);
        telemetry.update();
        sorter.setPosition(firstPos);
        firstColour = motif.get(0);
        secondColour = motif.get(1);
        //120 degrees = 0.067
        //240 degrees = 0.133
        if (firstColour.equals("P") && secondColour.equals("P")) {
            if (index == 0) {
                sorter.setPosition(secondPos);
                waitTime(1);
                push();
                sorter.setPosition(thirdPos);
                waitTime(1);
                push();

                sorter.setPosition(firstPos);
                waitTime(1);
                push();

                intake.clear();
            } else if (index == 1) {
                sorter.setPosition(firstPos);
                waitTime(1);
                push();

                sorter.setPosition(secondPos);
                waitTime(1);
                push();

                sorter.setPosition(thirdPos);
                waitTime(1);
                push();

                sorter.setPosition(firstPos);
                waitTime(1);
                push();

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
        } else if (firstColour.equals("P") && secondColour.equals("G")) {
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
        } else {
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

    //checks if
    void CheckArtifact() {
        PNum = 0;
        GNum = 0;
        for (int pointer = 0; pointer < 3; pointer++) {
            if (intake.get(pointer).equalsIgnoreCase("P")) {
                PNum++;
            }
            telemetry.addLine("# of Ps: " + PNum);
            telemetry.update();
        }

        for (int check = 0; check < 3; check++) {
            if (intake.get(check).equalsIgnoreCase("G")) {
                GNum++;
            }
            telemetry.addLine("# of G: " + GNum);
            telemetry.update();
        }

    }

    public void push () {
        pusher.setPosition(0.85);
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

    public void setLength() {
        length = intake.size();
        telemetry.addLine(Integer.toString(length));
    }

    public void clearList() {
        intake.clear();
    }

    public void setMotif(String sequence) {
        if (!motif.isEmpty()){
            motif.clear();
        }

        for (char c : sequence.toCharArray()){
            motif.add(String.valueOf(c));
        }
    }
    public void temporarySort(){
        if(pos == thirdPos){
            push();
            waitTime(0.5);

            sorter.setPosition(secondPos);
            waitTime(2);
            push();
            waitTime(0.5);

            sorter.setPosition(firstPos);
            waitTime(2);
            push();
            waitTime(0.5);
        }
        else if(pos == firstPos){
            push();
            waitTime(0.5);

            sorter.setPosition(secondPos);
            waitTime(2);
            push();
            waitTime(0.5);

            sorter.setPosition(thirdPos);
            waitTime(2);
            push();
            waitTime(0.5);

            sorter.setPosition(firstPos);
            waitTime(2);
        }
    }

}