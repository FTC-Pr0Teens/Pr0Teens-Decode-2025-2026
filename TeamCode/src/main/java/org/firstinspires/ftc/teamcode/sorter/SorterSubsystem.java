package org.firstinspires.ftc.teamcode.sorter;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware;

import java.util.ArrayList;

public class SorterSubsystem {
    public ArrayList<String> order;

    public ArrayList<String> intake;

    private ColorSensor colourSensor;
    private Servo turnTable;
    double greenDetected;

    private String firstColour;
    private String secondColour;

    public int index;
    double purple1Detected;

    double purple2Detected;

    private Hardware hw;
    private int length;


    public  SorterSubsystem(int gD, int pD1, int pD2, Hardware hw) {
        this.order = new ArrayList<>();
        this.intake = new ArrayList<>();

        this.hw = hw;

        this.greenDetected = gD;
        this.purple1Detected = pD1;
        this.purple2Detected = pD2;
        //length = intake.size();
        length = 0;
        this.colourSensor = hw.colourSensor;
        this.turnTable = hw.turnTable;


    }

    private void detectColour() throws InterruptedException {

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
        if (length <= 3) {
            if (blue > green) {
                telemetry.addData("Blue", colourSensor.blue());
                telemetry.update();

                telemetry.addLine("purple detected");
                if (purple1Detected == 1 && purple2Detected == 1) { //continues to add purple even if purple1Detected and purple2Detected == 1
                    telemetry.addLine("purple1Full"); //it does not output purpleFull
                    telemetry.update();
                } else if (purple1Detected == 0 || purple2Detected == 0) {
                    if (purple1Detected == 0) {
                        purple1Detected = 1;
                        telemetry.addLine("purple1Detected");
                        telemetry.update();
                        if (length == 0) {
                            turnTable.setPosition(0);
                            sleep(1000);
                            intake.add("purple");
                            telemetry.addLine(Integer.toString(length));
                            telemetry.update();
                        } else if (length == 1) {
                            turnTable.setPosition(0.5);
                            sleep(1000);
                            turnTable.setPosition(0);
                            sleep(1000);
                            intake.add("purple");
                            telemetry.addLine(Integer.toString(length));
                            telemetry.update();
                        } else {
                            turnTable.setPosition(1);
                            sleep(1000);
                            turnTable.setPosition(0);
                            sleep(1000);
                            intake.add("purple");
                            telemetry.addLine(Integer.toString(length));
                            telemetry.update();
                        }
                    } else if (purple2Detected == 0 && purple1Detected != 0) {
                        purple2Detected = 1;
                        telemetry.addLine("purple1Detected");
                        telemetry.update();
                        if (length == 0) {
                            turnTable.setPosition(0);
                            sleep(1000);
                            intake.add("purple");
                            telemetry.addLine(Integer.toString(length));
                            telemetry.update();
                        } else if (length == 1) {
                            turnTable.setPosition(0.5);
                            sleep(1000);
                            turnTable.setPosition(0);
                            sleep(1000);
                            intake.add("purple");
                            telemetry.addLine(Integer.toString(length));
                            telemetry.update();
                        } else {
                            turnTable.setPosition(1);
                            sleep(1000);
                            turnTable.setPosition(0);
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
                telemetry.addData("Green", colourSensor.green());
                telemetry.update();
                telemetry.addLine("green detected");
                if (greenDetected == 1) {
                    telemetry.addLine("greenFull");
                    telemetry.update();
                } else if (greenDetected == 0) {
                    greenDetected = greenDetected + 1;
                    if (length == 0) {
                        turnTable.setPosition(0); 
                        sleep(1000);
                        intake.add("green");
                        telemetry.addLine(Integer.toString(length));
                        telemetry.update();
                    } else if (length == 1) {
                        turnTable.setPosition(0.5);
                        sleep(1000);
                        intake.add("green");
                        telemetry.addLine(Integer.toString(length));
                        telemetry.update();
                    } else {
                        turnTable.setPosition(1);
                        sleep(1000);
                        intake.add("green");
                        telemetry.addLine(Integer.toString(length));
                        telemetry.update();
                    }
                }
            }
        }
    }

    private void sort() throws InterruptedException {
        index = intake.indexOf(new String("green"));
        turnTable.setPosition(0);
        firstColour = order.get(0);
        secondColour = order.get(1);
        if (firstColour.equals("purple") && secondColour.equals("purple")) {
            if (index == 0) {
                turnTable.setPosition(0.5);
                sleep(1000);
                turnTable.setPosition(1);
                sleep(1000);
                turnTable.setPosition(0);
                sleep(7000);
                intake.clear();
            } else if (index == 1) {
                turnTable.setPosition(0);//60 degrees
                sleep(1000);
                turnTable.setPosition(0.5);
                sleep(1000);
                turnTable.setPosition(1);
                sleep(1000);
                turnTable.setPosition(0);
                sleep(1000);
                intake.clear();

            } else if (index == 2) {
                turnTable.setPosition(0);//60 degrees
                sleep(1000);
                turnTable.setPosition(1);
                sleep(1000);
                turnTable.setPosition(0.5);
                sleep(1000);
                turnTable.setPosition(0);
                sleep(1000);
                intake.clear();
            }
        } else if (firstColour.equals("purple") && secondColour.equals("green")) {
            if (index == 0) {
                turnTable.setPosition(0.5);
                sleep(1000);
                turnTable.setPosition(0);
                sleep(1000);
                turnTable.setPosition(1);
                sleep(1000);
                turnTable.setPosition(0);
                sleep(1000);
                intake.clear();
            } else if (index == 1) {
                turnTable.setPosition(0);
                sleep(1000);
                turnTable.setPosition(1);
                sleep(1000);
                turnTable.setPosition(0.5);
                sleep(1000);
                turnTable.setPosition(0);
                sleep(500);
                intake.clear();
            } else if (index == 2) {
                turnTable.setPosition(0);
                sleep(1000);
                turnTable.setPosition(0.5);
                sleep(1000);
                turnTable.setPosition(1);
                sleep(1000);
                turnTable.setPosition(0);
                sleep(500);
                intake.clear();
            }
        } else { //if (firstColour.equals("green")) {
            telemetry.addData("index", index);
            telemetry.update();
            if (index == 0) {
                turnTable.setPosition(0);//60 degrees
                sleep(1000);
                turnTable.setPosition(0.5);
                sleep(1000);
                turnTable.setPosition(1);
                sleep(1000);
                turnTable.setPosition(0);
                sleep(1000);
                intake.clear();
            } else if (index == 1) {
                turnTable.setPosition(1);//60 degrees
                sleep(1000);
                turnTable.setPosition(0.5);
                sleep(1000);
                turnTable.setPosition(0);
                sleep(1000);
                intake.clear();
            } else if (index == 2) {
                turnTable.setPosition(0.5);//60 degrees
                sleep(1000);
                turnTable.setPosition(1);
                sleep(1000);
                turnTable.setPosition(0);
                sleep(1000);
                intake.clear();
            }
        }
    }

    public int getSize() {
        return intake.size();
    }
}
