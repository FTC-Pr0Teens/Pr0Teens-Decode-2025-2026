package org.firstinspires.ftc.teamcode.opmodes.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "servo")
public class servotester3 extends LinearOpMode {
    private Servo push;
    private double position;


    @Override

    public void runOpMode() throws InterruptedException {

        push = hardwareMap.get(Servo.class, "push");
        push.setDirection(Servo.Direction.REVERSE);
        waitForStart();


        while (opModeIsActive()) {
            push.setPosition(0.0);
            if(gamepad1.a){
                push.setPosition(0.5);
            }



        }
    }
}
