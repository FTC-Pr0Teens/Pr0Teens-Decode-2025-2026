package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommand;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;


@TeleOp(name = "TeleopSample", group = "TeleOp")
public class SampleTeleOpMode extends LinearOpMode {

    // opmodes should only own commands
    private GoBildaPinpointDriver odo;

    // Current pose estimates (in cm or degrees as appropriate)
    private MecanumCommand mecanumCommand;
    private LiftCommand liftCommand;

    private ElapsedTime timer;

    private ElapsedTime resetTimer;

    private Hardware hw;

    double heading;
    enum ROBOT_STATE{
        IDLE, SLOW
    }

    public ROBOT_STATE liftState = ROBOT_STATE.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        liftCommand = new LiftCommand(hw);

//        liftCommand = new LiftCommand(hw);
        resetTimer = new ElapsedTime();
        // Wait for start button to be pressed
        waitForStart();

        // Loop while OpMode is running
        while (opModeIsActive()) {
            telemetry.addLine("chaewon");
            heading = mecanumCommand.fieldOrientedMove(
                    - gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            processTelemetry();

            if (gamepad1.start){
                mecanumCommand.resetPinPointOdometry();
            }
            if(gamepad1.left_bumper == true) {
                liftCommand.handleIntake();


            }
            if (gamepad1.left_bumper == false){
                liftCommand.stopintake();
            }
            if(gamepad1.a){
                liftCommand.turn();
            }
            if(gamepad1.b){
                liftCommand.stopturn();
            }


        }

    }
    public void processTelemetry() {
        //add telemetry messages here
        mecanumCommand.deadReckoning();

        telemetry.addData("x: ", mecanumCommand.getOdoX());
        telemetry.addData("y: ", mecanumCommand.getOdoY());
        telemetry.addData("heading: ", heading);
        telemetry.update();
    }
}




