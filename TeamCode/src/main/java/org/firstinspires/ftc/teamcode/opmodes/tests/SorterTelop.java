package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.sorter.SorterSubsystem;

@TeleOp (name = "SortIntake (Blocks to Java)")

public class SorterTelop extends LinearOpMode {

    private Hardware hw;
    private SorterSubsystem sorterSubsystem;

    ArrayList<String> order = new ArrayList<>();
    ArrayList<String> intake = new ArrayList<>();

    String firstColour;
    String secondColour;
    private int index;
    double greenDetected = 0;
    double purple1Detected = 0;
    double purple2Detected = 0;
    int length = intake.size();

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = Hardware.getInstance(hardwareMap);

        sorterSubsystem = new SorterSubsystem(0,0,0, hw);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        hw.colourSensor.enableLed(true);





        waitForStart();
        while (opModeIsActive()) {
            index = intake.indexOf(new String("green"));

            double maxDegree = 300;
            double currentPosition = hw.turnTable.getPosition();
            double currentDegrees = currentPosition * maxDegree;

            int greenIndex = order.indexOf("green");

            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            double spin = -gamepad1.right_stick_y;

            // moves intake
            if(gamepad1.b == true){
                telemetry.addLine("buttonB pressed");
                telemetry.update();
                sorterSubsystem.detectColour();
            }
            if (gamepad1.a == true) {
                sorterSubsystem.sort();

            }
            if(gamepad1.x == true){
                intake.clear();
                telemetry.addLine(Integer.toString(length));
                telemetry.update();
            }
        }
    }
    private void detectColour () {

        int red = hw.colourSensor.red();
        int green = hw.colourSensor.green();
        int blue = hw.colourSensor.blue();
        int alpha = hw.colourSensor.alpha();

        int length = intake.size();

        telemetry.addData("intake: ", intake.size());

        telemetry.addData("Red", hw.colourSensor.red());
        telemetry.addData("Green", hw.colourSensor.green());
        telemetry.addData("Blue", hw.colourSensor.blue());
        telemetry.addData("Alpha", hw.colourSensor.alpha());

        telemetry.update();


        //purple
        if(length <= 3){
            if (blue > green) {
                telemetry.addData("Blue", hw.colourSensor.blue());
                telemetry.update();

                telemetry.addLine("purple detected");
                if (purple1Detected == 1 && purple2Detected == 1){ //continues to add purple even if purple1Detected and purple2Detected == 1
                    telemetry.addLine("purple1Full"); //it does not output purpleFull
                    telemetry.update();
                }

                else if (purple1Detected == 0 || purple2Detected == 0) {
                    if(purple1Detected == 0){
                        purple1Detected = 1;
                        telemetry.addLine("purple1Detected");
                        telemetry.update();
                        if (length == 0) {
                            hw.turnTable.setPosition(0);
                            sleep(1000);
                            intake.add("purple");
                            telemetry.addLine(Integer.toString(length));
                            telemetry.update();
                        } else if (length == 1) {
                            hw.turnTable.setPosition(0.5);
                            sleep(1000);
                            hw.turnTable.setPosition(0);
                            sleep(1000);
                            intake.add("purple");
                            telemetry.addLine(Integer.toString(length));
                            telemetry.update();
                        } else {
                            hw.turnTable.setPosition(1);
                            sleep(1000);
                            hw.turnTable.setPosition(0);
                            sleep(1000);
                            intake.add("purple");
                            telemetry.addLine(Integer.toString(length));
                            telemetry.update();
                        }
                    }
                    else if(purple2Detected == 0 && purple1Detected != 0){
                        purple2Detected = 1;
                        telemetry.addLine("purple1Detected");
                        telemetry.update();
                        if (length == 0) {
                            hw.turnTable.setPosition(0);
                            sleep(1000);
                            intake.add("purple");
                            telemetry.addLine(Integer.toString(length));
                            telemetry.update();
                        } else if (length == 1) {
                            hw.turnTable.setPosition(0.5);
                            sleep(1000);
                            hw.turnTable.setPosition(0);
                            sleep(1000);
                            intake.add("purple");
                            telemetry.addLine(Integer.toString(length));
                            telemetry.update();
                        } else {
                            hw.turnTable.setPosition(1);
                            sleep(1000);
                            hw.turnTable.setPosition(0);
                            sleep(1000);
                            intake.add("purple");
                            telemetry.addLine(Integer.toString(length));
                            telemetry.update();
                        }
                    }
                }
            }


//green
            if (green > blue) {
                telemetry.addData("Green", hw.colourSensor.green());
                telemetry.update();
                telemetry.addLine("green detected");
                if (greenDetected == 1) {
                    telemetry.addLine("greenFull");
                    telemetry.update();
                } else if(greenDetected == 0){
                    greenDetected = greenDetected + 1;
                    if (length == 0) {
                        hw.turnTable.setPosition(0);
                        sleep(1000);
                        intake.add("green");
                        telemetry.addLine(Integer.toString(length));
                        telemetry.update();
                    } else if (length == 1) {
                        hw.turnTable.setPosition(0.5);
                        sleep(1000);
                        intake.add("green");
                        telemetry.addLine(Integer.toString(length));
                        telemetry.update();
                    } else {
                        hw.turnTable.setPosition(1);
                        sleep(1000);
                        intake.add("green");
                        telemetry.addLine(Integer.toString(length));
                        telemetry.update();
                    }
                }
            }
        }
    }

    private void sort(){
        index = intake.indexOf(new String("green"));
        hw.turnTable.setPosition(0);
        firstColour = order.get(0);
        secondColour = order.get(1);
        if (firstColour.equals("purple") && secondColour.equals("purple")) {
            if (index == 0) {
                hw.turnTable.setPosition(0.5);
                sleep(1000);
                hw.turnTable.setPosition(1);
                sleep(1000);
                hw.turnTable.setPosition(0);
                sleep(7000);
                intake.clear();
            }
            else if (index == 1) {
                hw.turnTable.setPosition(0);//60 degrees
                sleep(1000);
                hw.turnTable.setPosition(0.5);
                sleep(1000);
                hw.turnTable.setPosition(1);
                sleep(1000);
                hw.turnTable.setPosition(0);
                sleep(1000);
                intake.clear();

            }
            else if (index == 2) {
                hw.turnTable.setPosition(0);//60 degrees
                sleep(1000);
                hw.turnTable.setPosition(1);
                sleep(1000);
                hw.turnTable.setPosition(0.5);
                sleep(1000);
                hw.turnTable.setPosition(0);
                sleep(1000);
                intake.clear();
            }
        }
        else if (firstColour.equals("purple") && secondColour.equals("green")) {
            if (index == 0) {
                hw.turnTable.setPosition(0.5);
                sleep(1000);
                hw.turnTable.setPosition(0);
                sleep(1000);
                hw.turnTable.setPosition(1);
                sleep(1000);
                hw.turnTable.setPosition(0);
                sleep(1000);
                intake.clear();
            }
            else if (index == 1) {
                hw.turnTable.setPosition(0);
                sleep(1000);
                hw.turnTable.setPosition(1);
                sleep(1000);
                hw.turnTable.setPosition(0.5);
                sleep(1000);
                hw.turnTable.setPosition(0);
                sleep(500);
                intake.clear();
            }
            else if (index == 2) {
                hw.turnTable.setPosition(0);
                sleep(1000);
                hw.turnTable.setPosition(0.5);
                sleep(1000);
                hw.turnTable.setPosition(1);
                sleep(1000);
                hw.turnTable.setPosition(0);
                sleep(500);
                intake.clear();
            }
        }
        else{ //if (firstColour.equals("green")) {
            telemetry.addData("index", index);
            telemetry.update();
            if (index == 0) {
                hw.turnTable.setPosition(0);//60 degrees
                sleep(1000);
                hw.turnTable.setPosition(0.5);
                sleep(1000);
                hw.turnTable.setPosition(1);
                sleep(1000);
                hw.turnTable.setPosition(0);
                sleep(500);
                intake.clear();
            }
            else if (index == 1) {
                hw.turnTable.setPosition(1);//60 degrees
                sleep(1000);
                hw.turnTable.setPosition(0.5);
                sleep(1000);
                hw.turnTable.setPosition(0);
                sleep(10000000);
                intake.clear();
            }
            else if (index == 2) {
                hw.turnTable.setPosition(0.5);//60 degrees
                sleep(1000);
                hw.turnTable.setPosition(1);
                sleep(1000);
                hw.turnTable.setPosition(0);
                sleep(1000);
                intake.clear();
            }
        }

    }
    public int getSize(){
        return intake.size();
    }
}