//package org.firstinspires.ftc.teamcode.opmodes.tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import java.util.ArrayList;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Hardware;
////import org.firstinspires.ftc.teamcode.subsystems.cameras.LogitechSubsystem;
////import org.firstinspires.ftc.teamcode.subsystems.sorting.SortingSubsystem;
//
//import dalvik.system.DelegateLastClassLoader;
//
//@TeleOp (name = "Sorting")
//
//public class Sorting extends LinearOpMode {
//    String motif;
//    ArrayList<String> intake = new ArrayList<>();
//    private DcMotor rb;
//    //private SortingSubsystem sortingSubsystem;
//    //private LogitechSubsystem logitechSubsystem;
//    private Hardware hw;
//    ElapsedTime sorterTimer = new ElapsedTime();;
//    int counter = 1;
//
//    @Override
//    public void runOpMode() {
//        hw = Hardware.getInstance(hardwareMap);
//        //
//        motif = "PPG";
//
//        //sortingSubsystem = new SortingSubsystem(hw, telemetry, motif);
//
//        intake.add("purple");
//        intake.add("green");
//        intake.add("purple");
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        hw.sorter.setPosition(0);
//        waitForStart();
//        sorterTimer.reset();
//        while (opModeIsActive()) {
//            // moves intake55
//            if (gamepad1.b) {
//                //sortingSubsystem.detectColour();
//            }
//            if (gamepad1.a && sorterTimer.milliseconds() >= 500) {
//                telemetry.addLine("running Sort code");
//                sorterTimer.reset();
//                //sortingSubsystem.temporarySort();
//
//                if (counter == 1) {
//                    hw.sorter.setPosition(0);
//                    //sorterSubsystem.sort();
//                } else if (counter == 2) {
//                    hw.sorter.setPosition(0.38);
//                    //sorterSubsystem.sort();
//                } else if (counter == 3) {
//                    hw.sorter.setPosition(0.78); //try 0.95
//                    //sorterSubsystem.sort();
//                }
//                counter++;
//                if (counter >= 4) {
//                    counter = 1;
//                }
//            }
//
//            if (gamepad1.y) {
////             sorterSubsystem.push();
//            }
//            if (gamepad1.x) {
//                sortingSubsystem.clearList();
//                sortingSubsystem.setLength();
//                telemetry.update();
//            }
//            telemetry.update();
//        }
//    }
//
//}