package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.sorter.SorterSubsystem;

@TeleOp (name = "SortingCode (Blocks to Java)")

public class Sorting extends LinearOpMode {

    private SorterSubsystem sorterSubsystem;
    private Hardware hw;
    ArrayList<String> motif;

    @Override
    public void runOpMode() {
        hw = Hardware.getInstance(hardwareMap);
        sorterSubsystem = new SorterSubsystem(hw, motif, telemetry);


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        hw.colourSensor.enableLed(true);

        waitForStart();
        while (opModeIsActive()) {
            // moves intake55
            if (gamepad1.b) {
                sorterSubsystem.detectColour();
            }
            if (gamepad1.a) {
                sorterSubsystem.sort();
            }
            if (gamepad1.y) {
                sorterSubsystem.push();
            }
            if (gamepad1.x) {
                sorterSubsystem.clearList();
                sorterSubsystem.setLength();
                telemetry.update();
            }
        }
    }

}