package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.sorter.SorterSubsystem;


@TeleOp(name = "TeleopSample", group = "TeleOp")
public class SampleTeleOpMode extends LinearOpMode {

    // opmodes should only own commands
    private MecanumCommand mecanumCommand;

    private SorterSubsystem sorterSubsystem;
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
        resetTimer = new ElapsedTime();
        while (opModeInInit()){
            telemetry.update();
        }

        // Wait for start button to be pressed
        waitForStart();

        // Loop while OpMode is running
        while (opModeIsActive()) {
            heading = mecanumCommand.fieldOrientedMove(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            processTelemetry();

            if (gamepad1.start){
                mecanumCommand.resetPinPointOdometry();
            }

            if (gamepad1.x){
                hw.intake.setPower(1);
            } else {
                hw.intake.setPower(0);
            }

            if (gamepad1.a){
                hw.shooter.setPower(1);
            } else {
                hw.shooter.setPower(0);
            }
        }

    }
    public void processTelemetry(){
        //add telemetry messages here
        telemetry.addData("resetTimer: ",  resetTimer.milliseconds());
        telemetry.addData("heading: ",  heading);
        telemetry.addLine("---------------------------------");

        telemetry.update();
    }
}