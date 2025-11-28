package org.firstinspires.ftc.teamcode.subsystems.sorting;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.cameras.LogitechSubsystem;
import org.firstinspires.ftc.teamcode.util.Artifact;

import java.util.ArrayList;

public class SortingSubsystem {
    ArrayList<String> motif;
    ArrayList<Artifact> intake;
    ColorSensor colourSensor;
    private Servo sorter;
    private Servo pusher;
    int length;
    int PNum;
    int GNum;
    private Hardware hw;

    private ElapsedTime timer;

    public Telemetry telemetry;
    private LogitechSubsystem logitechSubsystem;
    private static final double SORTER_FIRST_POS = 0.0;
    private static final double SORTER_SECOND_POS = 0.45;
    private static final double SORTER_THIRD_POS = 0.88;
    double pos;

    private static final double PUSHER_UP = 0.39;
    private static final double PUSHER_DOWN = 0.0;
    private static final double PUSHER_UP1 = 0.19;
    private static final double PUSHER_DOWN1 = 0;
    private static final long PUSHER_TIME = 750;
    private static final long WAIT_TIME = 2000;
    private final ElapsedTime pusherTimer = new ElapsedTime();
    private boolean isPusherUp = false;
    private final ElapsedTime sorterTimer = new ElapsedTime();

    private final ElapsedTime wait = new ElapsedTime();

    enum AUTO_STATE {
        SORT,
        PUSH_UP,
        PUSH_DOWN

    }

    AUTO_STATE autoState = AUTO_STATE.SORT;


    public SortingSubsystem(Hardware hw, Telemetry telemetry, String motif) {
        logitechSubsystem = new LogitechSubsystem(hw, "red");
        this.hw = hw;
        this.sorter = hw.sorter;
        this.pusher = hw.pusher;
        timer = new ElapsedTime();
        intake = new ArrayList<>();
        this.motif = new ArrayList<>();

        //  length = 0;
        setMotif(motif);
        this.telemetry = telemetry;
        pos = sorter.getPosition();
    }

    public void intake(String colour) {
        length = intake.size();
        if (length <= 3) {
            if (colour.equals("purple")) {
                PNum++;
            } else if (colour.equals("green")) {
                GNum++;
            }
            Artifact newArti = new Artifact(colour, length);
            intake.add(newArti);
        }
        if (length == 1) {
            hw.sorter.setPosition(SORTER_FIRST_POS);
        } else if (length == 2) {
            hw.sorter.setPosition(SORTER_SECOND_POS);
        } else {
            hw.sorter.setPosition(SORTER_THIRD_POS);
        }
    }


    public void outtake() {
        if (intake.size() == 3 && GNum == 1 && PNum == 2) {
            int motifIndex = 0;
            // Iterate backwards through intake to minimize sorter rotation
            // but follow motif order (0 to 2)
            for (int i = intake.size() - 1; i >= 0; i--) {
                Artifact arti = intake.get(i);
                if (arti == null) continue;
                // Check if artifact color matches motif in order
                if (arti.getColour().equals(motif.get(motifIndex))) {
                    motifIndex++;
                    outtakeArtifact(arti.getPosition());
                    intake.remove(i);
                }
            }
        } else {
            // Empty in any order
            for (int i = intake.size() - 1; i >= 0; i--) {
                Artifact arti = intake.get(i);
                if (arti == null) continue;

                outtakeArtifact(arti.getPosition());
                intake.remove(i);
            }
        }
    }


    public void outtakeArtifact(int position) {
        // macro here
        switch (autoState) {
            case SORT:
                wait.reset();

                if (wait.milliseconds() >= WAIT_TIME) {
                    sorter.setPosition(position);
                    autoState = AUTO_STATE.PUSH_UP;
                }
                break;

            case PUSH_UP:
                if (!isPusherUp) {
                    hw.pusher.setPosition(PUSHER_UP);
                    hw.pusher1.setPosition(PUSHER_UP1);
                    pusherTimer.reset();
                    isPusherUp = true;
                }
                autoState = AUTO_STATE.PUSH_DOWN;
                break;

            case PUSH_DOWN:
                if (isPusherUp && pusherTimer.milliseconds() >= PUSHER_TIME) {
                    hw.pusher.setPosition(PUSHER_DOWN);
                    hw.pusher1.setPosition(PUSHER_DOWN1);
                    isPusherUp = false;
                }
                break;
        }
    }

    //organizes artifacts on intake and adds the colour of each artifact to a list in order of entry
//    public void detectColour() { //change name to intake
//        telemetry.addLine("detectColour");
//        telemetry.update();
//        int red = colourSensor.red();
//        int green = colourSensor.green();
//        int blue = colourSensor.blue();
//        int alpha = colourSensor.alpha();
//
//        telemetry.addData("intake: ", intake.size());
//
//        telemetry.addData("Red", colourSensor.red());
//        telemetry.addData("G", colourSensor.green());
//        telemetry.addData("Blue", colourSensor.blue());
//        telemetry.addData("Alpha", colourSensor.alpha());
//        telemetry.update();
//        //P
//        if (length < 3) {
//            CheckArtifact();
//            if (blue > green) { //
//                telemetry.addLine("P detected");
//                telemetry.update();
//                if (length == 0) {
//                    sorter.setPosition(firstPos);
//                    waitTime(1);
//                    intake.add("P");
//                    telemetry.addLine(Integer.toString(length));
//                    telemetry.update();
//                } else if (length == 1) {
//                    sorter.setPosition(secondPos);
//                    waitTime(1);
//                    sorter.setPosition(firstPos);
//                    waitTime(1);
//                    intake.add("P");
//                    telemetry.addLine(Integer.toString(length));
//                    telemetry.update();
//                } else {
//                    sorter.setPosition(thirdPos);
//                    waitTime(1);
//                    sorter.setPosition(firstPos);
//                    waitTime(1);
//                    intake.add("P");
//                    telemetry.addLine(Integer.toString(length));
//                    telemetry.update();
//                }
//            }
//        }
//
//        if (green > blue) {
//            telemetry.addData("G", colourSensor.green());
//            telemetry.update();
//            telemetry.addLine("G detected");
//            if (length == 0) {
//                sorter.setPosition(firstPos);
//                waitTime(1);
//                intake.add("G");
//                telemetry.addLine(Integer.toString(length));
//                telemetry.update();
//            } else if (length == 1) {
//                sorter.setPosition(secondPos);
//                waitTime(1);
//                sorter.setPosition(firstPos);
//                waitTime(1);
//                intake.add("G");
//                telemetry.addLine(Integer.toString(length));
//                telemetry.update();
//            } else {
//                sorter.setPosition(thirdPos);
//                waitTime(1);
//                sorter.setPosition(firstPos);
//                waitTime(1);
//                intake.add("G");
//                telemetry.addLine(Integer.toString(length));
//                telemetry.update();
//            }
//        }
//    }

    //organizes balls on outtake by comparing intake list with the motif
//    public void sort() {
//        length = intake.size();
//        index = intake.indexOf(new String("G"));
//        telemetry.addData("index ", index);
//        telemetry.update();
//        sorter.setPosition(firstPos);
//        firstColour = motif.get(0);
//        secondColour = motif.get(1);
//        //120 degrees = 0.067
//        //240 degrees = 0.133
//        if (firstColour.equals("P") && secondColour.equals("P")) {
//            if (index == 0) {
//                sorter.setPosition(secondPos);
//                waitTime(1);
//                push();
//                sorter.setPosition(thirdPos);
//                waitTime(1);
//                push();
//
//                sorter.setPosition(firstPos);
//                waitTime(1);
//                push();
//
//                intake.remove(0);
//            } else if (index == 1) {
//                sorter.setPosition(firstPos);
//                waitTime(1);
//                push();
//
//                sorter.setPosition(secondPos);
//                waitTime(1);
//                push();
//
//                sorter.setPosition(thirdPos);
//                waitTime(1);
//                push();
//
//                sorter.setPosition(firstPos);
//                waitTime(1);
//                push();
//
//                intake.clear();
//
//            } else if (index == 2) {
//                sorter.setPosition(firstPos);//60 degrees
//                waitTime(1);
//                sorter.setPosition(thirdPos);
//                waitTime(1);
//                sorter.setPosition(secondPos);
//                waitTime(1);
//                sorter.setPosition(firstPos);
//                waitTime(1);
//                intake.clear();
//            }
//        } else if (firstColour.equals("P") && secondColour.equals("G")) {
//            if (index == 0) {
//                sorter.setPosition(secondPos);
//                waitTime(1);
//                sorter.setPosition(firstPos);
//                waitTime(1);
//                sorter.setPosition(thirdPos);
//                waitTime(1);
//                sorter.setPosition(firstPos);
//                waitTime(1);
//                intake.clear();
//            } else if (index == 1) {
//                sorter.setPosition(firstPos);
//                waitTime(1);
//                sorter.setPosition(thirdPos);
//                waitTime(1);
//                sorter.setPosition(secondPos);
//                waitTime(1);
//                sorter.setPosition(firstPos);
//                waitTime(1);
//                intake.clear();
//            } else if (index == 2) {
//                sorter.setPosition(firstPos);
//                waitTime(1);
//                sorter.setPosition(secondPos);
//                waitTime(1);
//                sorter.setPosition(thirdPos);
//                waitTime(1);
//                sorter.setPosition(firstPos);
//                waitTime(1);
//                intake.clear();
//            }
//        } else {
//            telemetry.addData("index", index);
//            telemetry.update();
//            if (index == 0) {
//                sorter.setPosition(firstPos);//60 degrees
//                waitTime(1);
//                sorter.setPosition(secondPos);
//                waitTime(1);
//                sorter.setPosition(thirdPos);
//                waitTime(1);
//                sorter.setPosition(firstPos);
//                waitTime(1);
//                intake.clear();
//            } else if (index == 1) {
//                sorter.setPosition(thirdPos);//60 desgrees
//                waitTime(1);
//                sorter.setPosition(secondPos);
//                waitTime(1);
//                sorter.setPosition(firstPos);
//                waitTime(1);
//                intake.clear();
//            } else if (index == 2) {
//                sorter.setPosition(secondPos);//60 degrees
//                waitTime(1);
//                sorter.setPosition(thirdPos);
//                waitTime(1);
//                sorter.setPosition(firstPos);
//                waitTime(1);
//                intake.clear();
//            }
//        }
//
//    }

    //checks if
//    void CheckArtifact() {
//        PNum = 0;
//        GNum = 0;
//        for (int pointer = 0; pointer < 3; pointer++) {
//            if (intake.get(pointer).equalsIgnoreCase("P")) {
//                PNum++;
//            }
//            telemetry.addLine("# of Ps: " + PNum);
//            telemetry.update();
//        }
//
//        for (int check = 0; check < 3; check++) {
//            if (intake.get(check).equalsIgnoreCase("G")) {
//                GNum++;
//            }
//            telemetry.addLine("# of G: " + GNum);
//            telemetry.update();
//        }
//
//    }

    public void push() {
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
        if (!motif.isEmpty()) {
            motif.clear();
        }

        for (char c : sequence.toCharArray()) {
            motif.add(String.valueOf(c));
        }
    }

    public void temporarySort() {
        if (pos == SORTER_THIRD_POS) {
            push();
            waitTime(0.5);

            sorter.setPosition(SORTER_SECOND_POS);
            waitTime(2);
            push();
            waitTime(0.5);

            sorter.setPosition(SORTER_FIRST_POS);
            waitTime(2);
            push();
            waitTime(0.5);
        } else if (pos == SORTER_FIRST_POS) {
            push();
            waitTime(0.5);

            sorter.setPosition(SORTER_SECOND_POS);
            waitTime(2);
            push();
            waitTime(0.5);

            sorter.setPosition(SORTER_THIRD_POS);
            waitTime(2);
            push();
            waitTime(0.5);

            sorter.setPosition(SORTER_FIRST_POS);
            waitTime(2);
        }
    }
}