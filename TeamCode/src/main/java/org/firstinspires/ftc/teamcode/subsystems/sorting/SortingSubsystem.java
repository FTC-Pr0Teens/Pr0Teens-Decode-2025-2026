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
        PUSH_DOWN,
        RESET

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

    public SortingSubsystem(Hardware hw){
        this.hw = hw;
        this.sorter = hw.sorter;
        this.pusher = hw.pusher;
        timer = new ElapsedTime();
    }

    public void pusherDown(){
        hw.pusher.setPosition(PUSHER_DOWN);
        hw.pusher1.setPosition(PUSHER_DOWN1);
    }

    public void pusherUp(){
        hw.pusher.setPosition(PUSHER_UP);
        hw.pusher1.setPosition(PUSHER_UP1);
    }

    int index = 1;
    public void rotate(){
        index++;
        switch (index){
            case 1:
                hw.sorter.setPosition(SORTER_FIRST_POS);
                break;
            case 2:
                hw.sorter.setPosition(SORTER_SECOND_POS);
                break;
            case 3:
                hw.sorter.setPosition(SORTER_THIRD_POS);
                break;
            case 4:
                index = 1;
                hw.sorter.setPosition(SORTER_FIRST_POS);
                break;
        }

    }

    public void init(){
        hw.sorter.setPosition(SORTER_FIRST_POS);
        pusherDown();
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

    int currentIndex = 1;
    public void outtakeArtifact(){
        switch (autoState) {
            case PUSH_UP:
                hw.pusher.setPosition(PUSHER_UP);
                hw.pusher1.setPosition(PUSHER_UP1);
                pusherTimer.reset();
                autoState = AUTO_STATE.PUSH_DOWN;
                break;

            case PUSH_DOWN:
                if (isPusherUp && pusherTimer.milliseconds() >= PUSHER_TIME) {
                    hw.pusher.setPosition(PUSHER_DOWN);
                    hw.pusher1.setPosition(PUSHER_DOWN1);
                    isPusherUp = false;
                }
                break;
            case RESET:


        }
    }

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