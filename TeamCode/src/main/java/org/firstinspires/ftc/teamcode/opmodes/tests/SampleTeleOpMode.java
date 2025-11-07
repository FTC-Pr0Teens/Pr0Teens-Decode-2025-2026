package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.odometry.PinPointOdometrySubsystem;


@TeleOp(name = "TeleopSample", group = "TeleOp")
public class SampleTeleOpMode extends LinearOpMode {

    // opmodes should only own commands
    private MecanumCommand mecanumCommand;
    private ElapsedTime timer;
    private Hardware hw;
    private ElapsedTime resetTimer;

    // --- Button Variables ---
    private boolean previousAState = false;
    private boolean previousXState = false;
    private boolean previousYState = false;

    // --- Intake/Outake Variables---
    private boolean isIntakeMotorOn = false;
    private boolean isOuttakeMotorOn = false;

    // --- Pusher Variables ---
    private static final double PUSHER_UP = 0.2;
    private static final double PUSHER_DOWN = 0;
    private static final long PUSHER_TIME = 750;
    private final ElapsedTime pusherTimer = new ElapsedTime();

    private boolean isPusherUp = false;

    // --- Sorter Variables ---
    private final ElapsedTime sorterTimer = new ElapsedTime();
    int sorterpos = 0;
    private static final double SORTER_FIRST_POS = 0.0;
    private static final double SORTER_SECOND_POS = 0.42;
    private static final double SORTER_THIRD_POS = 0.88;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        resetTimer = new ElapsedTime();
        hw.pusher.setPosition(PUSHER_DOWN);
        hw.sorter.setPosition(0);


        // Wait for start button to be pressed
        waitForStart();

        // Loop while OpMode is running
        while (opModeIsActive()) {
            mecanumCommand.processOdometry();
            mecanumCommand.fieldOrientedMove(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            processTelemetry();

            if (gamepad1.start){
                mecanumCommand.resetPinPointOdometry();
            }

            // --- Intake toggle on A (edge) ---
            boolean currentAState = gamepad1.a;
            if (currentAState && !previousAState) {
                isIntakeMotorOn = !isIntakeMotorOn;
                hw.intake.setPower(isIntakeMotorOn ? 0.8 : 0.0);
            }
            previousAState = currentAState;


            // --- Pusher pulse on Y (edge) ---
            boolean currentYState = gamepad1.y;
            if (currentYState && !previousYState) {
                // Start pulse only if not already pulsing
                if (!isPusherUp) {
                    hw.pusher.setPosition(PUSHER_UP);
                    pusherTimer.reset();
                    isPusherUp = true;
                }
            }
            previousYState = currentYState;

            // Pusher
            if (isPusherUp && pusherTimer.milliseconds() >= PUSHER_TIME) {
                hw.pusher.setPosition(PUSHER_DOWN);
                isPusherUp = false;
            }

            // Outtake
            boolean currentXState = gamepad1.x;
            if (currentXState && !previousXState) {
                isOuttakeMotorOn = !isOuttakeMotorOn;
                hw.shooter.setPower(isOuttakeMotorOn ? 1.0 : 0.0);
            }
            previousXState = currentXState;


            if (gamepad1.b && sorterTimer.milliseconds() > 500 && !isPusherUp){
                sorterTimer.reset();
                if (sorterpos == 0) {
                    hw.sorter.setPosition(SORTER_FIRST_POS);//60 degrees
                }
                else if (sorterpos == 1) {
                    hw.sorter.setPosition(SORTER_SECOND_POS);//60 degrees
                }
                else if (sorterpos == 2) {
                    hw.sorter.setPosition(SORTER_THIRD_POS);//60 degrees
                }
                sorterpos = (sorterpos+1)%3;
            }

        }

    }
    public void processTelemetry(){
        //add telemetry messages here
        telemetry.addData("resetTimer: ",  resetTimer.milliseconds());
        telemetry.addLine("---------------------------------");
        telemetry.addData("X", mecanumCommand.getX());
        telemetry.addData("Y", mecanumCommand.getY());
        telemetry.addData("Pusher ON", isPusherUp);
        telemetry.addData("sorterpos", sorterpos);
        telemetry.update();
    }
}