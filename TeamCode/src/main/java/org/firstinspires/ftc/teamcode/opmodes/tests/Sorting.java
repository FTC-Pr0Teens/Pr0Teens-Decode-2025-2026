package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.sorter.SorterSubsystem;

import dalvik.system.DelegateLastClassLoader;

@TeleOp (name = "Sorting")

public class Sorting extends LinearOpMode {
    ArrayList<String> order = new ArrayList<>();
    ArrayList<String> intake = new ArrayList<>();
    private DcMotor rb;
    private SorterSubsystem sorterSubsystem;
    private Hardware hw;
    ArrayList<String> motif;

    ElapsedTime sorterTimer = new ElapsedTime();;
int counter = 0;
    @Override
    public void runOpMode() {
        hw = Hardware.getInstance(hardwareMap);
        sorterSubsystem = new SorterSubsystem(hw, motif, telemetry);

        order.add("purple");
        order.add("purple");
        order.add("green");

        intake.add("purple");
        intake.add("green");
        intake.add("purple");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        hw.sorter.setPosition(0);
        waitForStart();
        sorterTimer.reset();
        while (opModeIsActive()) {
            // moves intake55
            if (gamepad1.b) {
                sorterSubsystem.detectColour();
            }
            if (gamepad1.a && sorterTimer.milliseconds() >= 2000) {
                telemetry.addLine("running Sortcode");
               sorterTimer.reset();

                if (counter == 1) {
                    hw.sorter.setPosition(0);
                    //sorterSubsystem.sort();
                }
                if (counter == 2) {
                    hw.sorter.setPosition(0.42);
                    //sorterSubsystem.sort();

                }
                if (counter == 3) {
                    hw.sorter.setPosition(0.88);
                    //sorterSubsystem.sort();

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
            telemetry.update();
        }
    }

}