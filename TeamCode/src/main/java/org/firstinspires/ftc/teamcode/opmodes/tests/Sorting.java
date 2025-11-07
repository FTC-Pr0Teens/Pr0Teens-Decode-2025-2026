package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.sorter.SorterSubsystem;

import dalvik.system.DelegateLastClassLoader;

@TeleOp (name = "SortingCode (Blocks to Java)")

public class Sorting extends LinearOpMode {


    private SorterSubsystem sorterSubsystem;
    private Hardware hw;
    ArrayList<String> motif;

    ElapsedTime sorterTimer = new ElapsedTime();;
int counter = 0;
    @Override
    public void runOpMode() {
        hw = Hardware.getInstance(hardwareMap);
        sorterSubsystem = new SorterSubsystem(hw, motif, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        hw.sorter.setPosition(0);
        waitForStart();
        while (opModeIsActive()) {
            // moves intake55
            if (gamepad1.b) {
                sorterSubsystem.detectColour();
            }
            if (gamepad1.a && sorterTimer.milliseconds() >= 2000) {

                sorterTimer.reset();

                if (counter == 1) {
                    hw.sorter.setPosition(0);
                }
                if (counter == 2) {
                    hw.sorter.setPosition(0.42);
                }
                if (counter == 3) {
                    hw.sorter.setPosition(0.88);
                }
                if (counter == 4) {
                    counter = 1;
                }
            }

            if (gamepad1.y) {
//                sorterSubsystem.push();
            }
            if (gamepad1.x) {
                sorterSubsystem.clearList();
                sorterSubsystem.setLength();
                telemetry.update();
            }
        }
    }

}