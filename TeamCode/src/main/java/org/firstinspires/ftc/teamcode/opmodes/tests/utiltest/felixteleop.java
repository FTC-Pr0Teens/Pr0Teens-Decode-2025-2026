package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommand;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
@TeleOp(name = "chaewon teleop")
public class felixteleop extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private LiftCommand liftCommand;
    private ElapsedTime timer;

    private ElapsedTime resetTimer;

    //OpModes should create the hardware object
    private Hardware hw;


@Override
public void runOpMode() throws InterruptedException {
    hw = Hardware.getInstance(hardwareMap);
    mecanumCommand = new MecanumCommand(hw);
    liftCommand  = new LiftCommand(hw);
    resetTimer = new ElapsedTime();
    waitForStart();

    // Loop while OpMode is running
    while (opModeIsActive()) {





    }






        processTelemetry();

        if (gamepad1.start){
            mecanumCommand.resetPinPointOdometry();

        }
    }



    public void processTelemetry(){
        //add telemetry messages here
        telemetry.addData("resetTimer: ",  resetTimer.milliseconds());
        telemetry.addLine("---------------------------------");

        telemetry.update();
    }
}